#include "source.h"

int Source::rec_num = 0;

void Source::set_antenna(std::string ant)
{
  antenna = ant;

  if (driver == "usrp") {
    BOOST_LOG_TRIVIAL(info) << "Setting antenna to [" << antenna << "]";
    cast_to_usrp_sptr(source_block)->set_antenna(antenna, 0);
  }
}

std::string Source::get_antenna() {
  return antenna;
}

void Source::set_qpsk_mod(bool m) {
  qpsk_mod = m;
}

bool Source::get_qpsk_mod() {
  return qpsk_mod;
}

void Source::set_silence_frames(int m) {
  silence_frames = m;
}

int Source::get_silence_frames() {
  return silence_frames;
}

void Source::set_fsk_gain(double r) {
  fsk_gain = r;
}

double Source::get_fsk_gain() {
  return fsk_gain;
}

void Source::set_analog_levels(double r) {
  analog_levels = r;
}

double Source::get_analog_levels() {
  return analog_levels;
}

void Source::set_digital_levels(double r) {
  digital_levels = r;
}

double Source::get_digital_levels() {
  return digital_levels;
}

double Source::get_min_hz() {
  return min_hz;
}

double Source::get_max_hz() {
  return max_hz;
}

double Source::get_center() {
  return center;
}

double Source::get_rate() {
  return rate;
}

std::string Source::get_driver() {
  return driver;
}

std::string Source::get_device() {
  return device;
}

void Source::set_error(double e) {
  error = e;
}

double Source::get_error() {
  return error;
}

void Source::set_mix_gain(int b)
{
  if (driver == "osmosdr") {
    mix_gain = b;
    cast_to_osmo_sptr(source_block)->set_gain(mix_gain, "MIX", 0);
    BOOST_LOG_TRIVIAL(info) << "MIX Gain set to: " << cast_to_osmo_sptr(source_block)->get_gain("MIX");
  }
}

int Source::get_mix_gain() {
  if (driver == "osmosdr") {
    mix_gain = cast_to_osmo_sptr(source_block)->get_gain("MIX", 0);
  }
  return mix_gain;
}

void Source::set_lna_gain(int b)
{
  if (driver == "osmosdr") {
    lna_gain = b;
    cast_to_osmo_sptr(source_block)->set_gain(lna_gain, "LNA", 0);
    BOOST_LOG_TRIVIAL(info) << "LNA Gain set to: " << cast_to_osmo_sptr(source_block)->get_gain("LNA");
  }
}

int Source::get_lna_gain() {
  if (driver == "osmosdr") {
    lna_gain = cast_to_osmo_sptr(source_block)->get_gain("LNA", 0);
  }
  return lna_gain;
}

void Source::set_bb_gain(int b)
{
  if (driver == "osmosdr") {
    bb_gain = b;
    cast_to_osmo_sptr(source_block)->set_bb_gain(bb_gain);
    BOOST_LOG_TRIVIAL(info) << "BB Gain set to: " << cast_to_osmo_sptr(source_block)->get_gain("BB");
  }
}

int Source::get_bb_gain() {
  return bb_gain;
}

void Source::set_gain(int r)
{
  if (driver == "osmosdr") {
    gain = r;
    cast_to_osmo_sptr(source_block)->set_gain(gain);
    BOOST_LOG_TRIVIAL(info) << "Gain set to: " << cast_to_osmo_sptr(source_block)->get_gain();
  }

  if (driver == "usrp") {
    gain = r;
    cast_to_usrp_sptr(source_block)->set_gain(gain);
  }
}

int Source::get_gain() {
  return gain;
}

void Source::set_if_gain(int i)
{
  if (driver == "osmosdr") {
    if_gain = i;
    cast_to_osmo_sptr(source_block)->set_if_gain(if_gain);
    BOOST_LOG_TRIVIAL(info) << "IF Gain set to: " << cast_to_osmo_sptr(source_block)->get_gain("IF");
  }
}

void Source::set_freq_corr(double p)
{
  ppm = p;

  if (driver == "osmosdr") {
    cast_to_osmo_sptr(source_block)->set_freq_corr(ppm);
  }
}

int Source::get_if_gain() {
  return if_gain;
}

void Source::set_squelch_db(double s) {
  squelch_db = s;
}

double Source::get_squelch_db() {
  return squelch_db;
}


analog_recorder_sptr Source::create_conventional_recorder(gr::top_block_sptr tb) {

    analog_recorder_sptr log = make_analog_recorder(this);

    analog_recorders.push_back(log);
    tb->connect(source_block, 0, log, 0);
    return log;
}

p25_recorder_sptr Source::create_conventionalP25_recorder(gr::top_block_sptr tb) {

    p25_recorder_sptr log = make_p25_recorder(this);

    digital_recorders.push_back(log);
    tb->connect(source_block, 0, log, 0);
    return log;
}

void Source::create_analog_recorders(gr::top_block_sptr tb, int r) {
  max_analog_recorders = r;

  for (int i = 0; i < max_analog_recorders; i++) {
    analog_recorder_sptr log = make_analog_recorder(this);
    log->channel_port = channel_port_counter++;
    analog_recorders.push_back(log);
    tb->connect(channelizer, log->channel_port, log, 0);
  }
}

Recorder * Source::get_analog_recorder(int priority)
{
  if (priority > 99) {
    BOOST_LOG_TRIVIAL(info) << "\t\tNot recording because of priority";
    return NULL;
  }

  for (std::vector<analog_recorder_sptr>::iterator it = analog_recorders.begin();
       it != analog_recorders.end(); it++) {
    analog_recorder_sptr rx = *it;

    if (rx->get_state() == inactive)
    {
      return (Recorder *)rx.get();

      break;
    }
  }
  BOOST_LOG_TRIVIAL(info) << "[ " << driver << " ] No Analog Recorders Available";
  return NULL;
}

void Source::create_digital_recorders(gr::top_block_sptr tb, int r) {
  max_digital_recorders = r;

  for (int i = 0; i < max_digital_recorders; i++) {
    p25_recorder_sptr log = make_p25_recorder(this);
    log->num = rec_num++;
    log->channel_port = channel_port_counter++;
    digital_recorders.push_back(log);
    tb->connect(channelizer, log->channel_port, log, 0);
  }
}

int Source::freq_to_channel(double freq) {
  unsigned int channel = 0;
  double offset = freq - center;
  int channel_offset = floor(offset / channel_width);
  if (channel_offset < 0) {
    channel = channels+channel_offset;
  }
  else {
    channel = channel_offset;
  }
  return channel;
}

void Source::update_channel_map(int channel_port, double freq) {
  channel_map.resize(channel_port_counter);
  try {
    channel_map.at(channel_port) = freq_to_channel(freq));
  }
  catch(std::out_of_range o) {
    BOOST_LOG_TRIVIAL(error) << "Error mapping channel frequency" << o.what() << std::endl;
  }
  channelizer->set_channel_map(channel_map);
}

void Source::create_channelizer(gr::top_block_sptr tb) {
  long samp_rate = actual_rate;
  channel_width = 12500;
  float oversample_rate = 1.0;
  double transition_width = 10000; // Pretty loose, but assume no adjacent channels to save CPU.
  double attenuation = 60;
  channels = floor(samp_rate / channel_width);
  channel_map.reserve(channels);
  double channel_rate = samp_rate / channels;

  std::vector<float> filter_taps;
  filter_taps = gr::filter::firdes::low_pass_2(1.0, samp_rate, channel_width/2, transition_width, attenuation, gr::filter::firdes::WIN_HANN);
  gr::blocks::stream_to_streams::sptr s2ss_block = gr::blocks::stream_to_streams::make(sizeof(gr_complex), channels);
  channelizer = gr::filter::pfb_channelizer_ccf::make(channels, filter_taps, oversample_rate);
  channelizer->set_tag_propagation_policy(gr::block::tag_propagation_policy_t(0));  // I want the two weeks of my life back.

  BOOST_LOG_TRIVIAL(info) << "Channelizer channels: " << channels << "Channel width: " << channel_width <<  " Oversample rate: " << oversample_rate << " Channel rate: " << channel_rate;
  BOOST_LOG_TRIVIAL(info) << "Channelizer taps: " << filter_taps.size();

  tb->connect(source_block, 0, s2ss_block, 0);
  for (int i=0; i < channels; ++i) {
    tb->connect(s2ss_block, i, channelizer, i);
  }
}

void Source::create_debug_recorders(gr::top_block_sptr tb, int r) {
  max_debug_recorders = r;

  for (int i = 0; i < max_debug_recorders; i++) {
    debug_recorder_sptr log = make_debug_recorder(this);
    log->channel_port = channel_port_counter++;
    debug_recorders.push_back(log);
    tb->connect(channelizer, log->channel_port, log, 0);
  }
}

Recorder * Source::get_debug_recorder()
{
  for (std::vector<debug_recorder_sptr>::iterator it = debug_recorders.begin();
       it != debug_recorders.end(); it++) {
    debug_recorder_sptr rx = *it;

    if (rx->get_state() == inactive)
    {
      return (Recorder *)rx.get();

      break;
    }
  }
  return NULL;
}

void Source::print_recorders() {
  BOOST_LOG_TRIVIAL(info) << "[ " << device <<  " ]  ";

  for (std::vector<p25_recorder_sptr>::iterator it = digital_recorders.begin();
       it != digital_recorders.end(); it++) {
    p25_recorder_sptr rx = *it;

    BOOST_LOG_TRIVIAL(info) << "[ " << rx->get_num() << " ] State: " << rx->get_state();
  }
}

void Source::tune_digital_recorders() {
  for (std::vector<p25_recorder_sptr>::iterator it = digital_recorders.begin(); it != digital_recorders.end(); it++) {
    p25_recorder_sptr rx = *it;

    if (rx->get_state() == active)
    {
      rx->autotune();
    }
  }
}

int Source::get_num_available_recorders() {
  int num_available_recorders = 0;

  for (std::vector<p25_recorder_sptr>::iterator it = digital_recorders.begin();
       it != digital_recorders.end(); it++) {
    p25_recorder_sptr rx = *it;

    if (rx->get_state() == inactive)
    {
      num_available_recorders++;
    }
  }
  return num_available_recorders;
}

Recorder * Source::get_digital_recorder(int priority)
{
  // int num_available_recorders = get_num_available_recorders();
  // BOOST_LOG_TRIVIAL(info) << "\tTG Priority: "<< priority << " Available
  // Digital Recorders: " <<num_available_recorders;

  if (priority > 99) { // num_available_recorders) { // a low priority is bad.
                       // You need atleast the number of availalbe recorders to
                       // your priority
    // BOOST_LOG_TRIVIAL(info) << "Not recording because of priority";
    return NULL;
  }


  for (std::vector<p25_recorder_sptr>::iterator it = digital_recorders.begin();
       it != digital_recorders.end(); it++) {
    p25_recorder_sptr rx = *it;

    if (rx->get_state() == inactive)
    {
      return (Recorder *)rx.get();

      break;
    }
  }
  BOOST_LOG_TRIVIAL(info) << "[ " << device <<  " ] No Digital Recorders Available";

  for (std::vector<p25_recorder_sptr>::iterator it = digital_recorders.begin();
       it != digital_recorders.end(); it++) {
    p25_recorder_sptr rx = *it;
    BOOST_LOG_TRIVIAL(info) << "[ " << rx->get_num() << " ] State: " << rx->get_state() << " Freq: " << rx->get_freq();
  }
  return NULL;
}

gr::basic_block_sptr Source::get_src_block() {
  return source_block;
}

Config  * Source::get_config() {
  return config;
}

Source::Source(double c, double r, double e, std::string drv, std::string dev, Config *cfg)
{
  rate   = r;
  center = c;
  error  = e;
  min_hz = center - (rate / 2);
  max_hz = center + (rate / 2);
  driver = drv;
  device = dev;
  config = cfg;
  channel_port_counter = 0;

  if (driver == "osmosdr") {
    osmosdr::source::sptr osmo_src;
    std::vector<std::string> gain_names;
    if (dev == "") {
      BOOST_LOG_TRIVIAL(info) << "Source Device not specified";
      osmo_src = osmosdr::source::make();
    } else {
      std::ostringstream msg;

      if (isdigit(dev[0])) {  // Assume this is a serial number and fail back
                              // to using rtl as default
        msg << "rtl=" << dev; // <<  ",buflen=32764,buffers=8";
        BOOST_LOG_TRIVIAL(info) <<
          "Source device name missing, defaulting to rtl device";
      } else {
        msg << dev; // << ",buflen=32764,buffers=8";
      }
      BOOST_LOG_TRIVIAL(info) << "Source Device: " << msg.str();
      osmo_src = osmosdr::source::make(msg.str());
    }
    BOOST_LOG_TRIVIAL(info) << "SOURCE TYPE OSMOSDR (osmosdr)";
    BOOST_LOG_TRIVIAL(info) << "Setting sample rate to: " << rate;
    osmo_src->set_sample_rate(rate);
    actual_rate = osmo_src->get_sample_rate();
    BOOST_LOG_TRIVIAL(info) << "Actual sample rate: " << actual_rate;
    BOOST_LOG_TRIVIAL(info) << "Tunning to " << center + error << "hz";
    osmo_src->set_center_freq(center + error, 0);
    gain_names = osmo_src->get_gain_names();
    std::string gain_list;
    for (std::vector<std::string>::iterator it = gain_names.begin(); it != gain_names.end(); it++) {
       std::string gain_name = *it;
       osmosdr::gain_range_t range = osmo_src->get_gain_range(gain_name);
       std::vector<double> gains = range.values();
       std::string gain_opt_str;
       for (std::vector<double>::iterator gain_it = gains.begin(); gain_it != gains.end(); gain_it++) {
         double gain_opt =  *gain_it;
         std::ostringstream ss;
         //gain_opt = floor(gain_opt * 10) / 10;
         ss << gain_opt << " ";

         gain_opt_str += ss.str();
       }
       BOOST_LOG_TRIVIAL(info) << "Gain Stage: " << gain_name << " supported values: " <<  gain_opt_str;
    }
    source_block = osmo_src;
  }

  if (driver == "usrp") {
    gr::uhd::usrp_source::sptr usrp_src;
    usrp_src = gr::uhd::usrp_source::make(device, uhd::stream_args_t("fc32"));

    BOOST_LOG_TRIVIAL(info) << "SOURCE TYPE USRP (UHD)";

    BOOST_LOG_TRIVIAL(info) << "Setting sample rate to: " << rate;
    usrp_src->set_samp_rate(rate);
    actual_rate = usrp_src->get_samp_rate();
    BOOST_LOG_TRIVIAL(info) << "Actual sample rate: " << actual_rate;
    BOOST_LOG_TRIVIAL(info) << "Tunning to " << center + error << "hz";
    usrp_src->set_center_freq(center + error, 0);
    source_block = usrp_src;
  }
}
