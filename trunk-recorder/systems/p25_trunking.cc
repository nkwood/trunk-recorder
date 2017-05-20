
#include "p25_trunking.h"
#include <boost/log/trivial.hpp>


p25_trunking_sptr make_p25_trunking(double freq, double center, long s,  gr::msg_queue::sptr queue, bool qpsk, int sys_num)
{
  return gnuradio::get_initial_sptr(new p25_trunking(freq, center, s, queue, qpsk, sys_num));
}

p25_trunking::p25_trunking(double f, double c, long s, gr::msg_queue::sptr queue, bool qpsk, int sys_num)
  : gr::hier_block2("p25_trunking",
                    gr::io_signature::make(1, 1, sizeof(gr_complex)),
                    gr::io_signature::make(0, 0, sizeof(float)))
{

  this->sys_num = sys_num;
  chan_freq    = f;
  center_freq  = c;
  long samp_rate = s;

  double offset = chan_freq - center_freq;

  double symbol_rate         = 4800;
  double samples_per_symbol  = 6;
  double system_channel_rate = 12500; // symbol_rate * samples_per_symbol;
  double symbol_deviation    = 600.0;
  int initial_decim      = floor(samp_rate / 240000);
  double initial_rate = double(samp_rate) / double(initial_decim);
  int decim = floor(initial_rate / system_channel_rate);
  double resampled_rate = double(initial_rate) / double(decim);
  qpsk_mod = qpsk;


  double channel_width = 12500;
  double transition_width = 1000;
  double attenuation = 60;
  unsigned int channels = floor(samp_rate / channel_width); // 800

  int channel_offset = floor(offset / channel_width);
  unsigned int channel = 0;
  if (channel_offset < 0) {
    channel = channels+channel_offset;
  }
  else {
    channel = channel_offset;
  }
  double channel_rate = samp_rate / channels;

  float oversample_rate = 1.0; // float(channels)/71.0;

  std::vector<float> sym_taps;
  const double pi = M_PI; // boost::math::constants::pi<double>();

  double gain_mu      = 0.025;
  double costas_alpha = 0.04;
  double bb_gain      = 1.0;

  baseband_amp = gr::blocks::multiply_const_ff::make(bb_gain);

  std::vector<float> filter_taps;
  filter_taps = gr::filter::firdes::low_pass_2(1.0, samp_rate, channel_width/2, transition_width, attenuation, gr::filter::firdes::WIN_HANN);

  gr::blocks::stream_to_streams::sptr s2ss_block = gr::blocks::stream_to_streams::make(sizeof(gr_complex), channels);

  // BOOST_LOG_TRIVIAL(info) << "P25 Trunking Channels: " << channels << " Channel: " << channel <<  "Oversample rate: " << oversample_rate;

  //gr::filter::pfb_decimator_ccf::sptr pf_decimator = gr::filter::pfb_decimator_ccf::make(channels, filter_taps, channel);


  gr::filter::pfb_channelizer_ccf::sptr pf_channelizer = gr::filter::pfb_channelizer_ccf::make(channels, filter_taps, oversample_rate);
  

  std::vector<int> channel_map;
  channel_map.assign(1, channel);
  BOOST_LOG_TRIVIAL(info) << "Channel map: " << channel_map[0];
  pf_channelizer->set_channel_map(channel_map);

  // inital_lpf_taps = gr::filter::firdes::low_pass_2(1.0, samp_rate, 96000, 25000, 100, gr::filter::firdes::WIN_HANN);

  // std::vector<gr_complex> dest(inital_lpf_taps.begin(), inital_lpf_taps.end());

  // prefilter = make_freq_xlating_fft_filter(initial_decim, dest, offset, samp_rate);

  // channel_lpf_taps = gr::filter::firdes::low_pass_2(1.0, initial_rate, 7250, 1500, 100, gr::filter::firdes::WIN_HANN);

//  channel_lpf_taps = gr::filter::firdes::low_pass_2(1.0, resampled_rate, 6000, 1500, 100, gr::filter::firdes::WIN_HANN);
  // channel_lpf      =  gr::filter::fft_filter_ccf::make(decim, channel_lpf_taps);
  // double arb_rate  = (double(system_channel_rate) / resampled_rate);

  double post_channel_rate = samp_rate/channels*oversample_rate;

  // gr::filter::rational_resampler_base_ccc::sptr rational_resampler = gr::filter::rational_resampler_base_ccc::make(system_channel_rate, post_channel_rate, []);


  double arb_rate  = system_channel_rate / post_channel_rate;

  double arb_size  = 32;
  double arb_atten = 100;
  BOOST_LOG_TRIVIAL(info) << "P25 Trunking Sample Rate: " << samp_rate << " System Rate: " << system_channel_rate << " Post Channel Rate: " << post_channel_rate;
  BOOST_LOG_TRIVIAL(info) << "P25 Trunking Channels: " << channels  << " Channel: " << channel; 
  BOOST_LOG_TRIVIAL(info) << "P25 Trunking Taps: " << filter_taps.size();

  // gr::blocks::null_sink::sptr null_sinks[163];
  // for (int i=0; i<channels; ++i) {
  //   null_sinks[i] = gr::blocks::null_sink::make (sizeof (gr_complex));
  // }

  // gr::blocks::null_sink::sptr my_null_sink = gr::blocks::null_sink::make (sizeof (gr_complex));
  // BOOST_LOG_TRIVIAL(info) << "\t P25 Recorder Initial Rate: "<< initial_rate << " Resampled Rate: " << resampled_rate  << " Initial Decimation: " << initial_decim << " Decimation: " << decim << " System Rate: " << system_channel_rate << " ARB Rate: " << arb_rate;

  // Create a filter that covers the full bandwidth of the output signal

  // If rate >= 1, we need to prevent images in the output,
  // so we have to filter it to less than half the channel
  // width of 0.5.  If rate < 1, we need to filter to less
  // than half the output signal's bw to avoid aliasing, so
  // the half-band here is 0.5*rate.
//   double percent = 0.80;

//   if (arb_rate <= 1) {
//     double halfband = 0.5 * arb_rate;
//     double bw       = percent * halfband;
//     double tb       = (percent / 2.0) * halfband;


//     // As we drop the bw factor, the optfir filter has a harder time converging;
//     // using the firdes method here for better results.
//     arb_taps = gr::filter::firdes::low_pass_2(arb_size, arb_size, bw, tb, arb_atten,
//                                               gr::filter::firdes::WIN_BLACKMAN_HARRIS);
// //    gr::filter::rational_resampler_base_ccc::sptr rational_resampler = gr::filter::rational_resampler_base_ccc::make(system_channel_rate, post_channel_rate, []);

//     // double tap_total = inital_lpf_taps.size() + channel_lpf_taps.size() + arb_taps.size();
//     BOOST_LOG_TRIVIAL(info) << "P25 Trunking " << " ARB Taps: " << arb_taps.size();
//   } else {
//     BOOST_LOG_TRIVIAL(error) << "Something is probably wrong! Resampling rate too low";
//   }

//   arb_resampler = gr::filter::pfb_arb_resampler_ccf::make(arb_rate, arb_taps);


  agc = gr::analog::feedforward_agc_cc::make(16, 1.0);

  double omega      = double(system_channel_rate) / double(symbol_rate);
  double gain_omega = 0.1  * gain_mu * gain_mu;
  double alpha      = costas_alpha;
  double beta       = 0.125 * alpha * alpha;
  double fmax       = 2400; // Hz
  fmax = 2 * pi * fmax / double(system_channel_rate);

  costas_clock = gr::op25_repeater::gardner_costas_cc::make(omega, gain_mu, gain_omega, alpha,  beta, fmax, -fmax);


  // Perform Differential decoding on the constellation
  diffdec = gr::digital::diff_phasor_cc::make();

  // take angle of the difference (in radians)
  to_float = gr::blocks::complex_to_arg::make();

  // convert from radians such that signal is in -3/-1/+1/+3
  rescale = gr::blocks::multiply_const_ff::make((1 / (pi / 4)));

  // fm demodulator (needed in fsk4 case)
  // double fm_demod_gain = system_channel_rate / (2.0 * pi * symbol_deviation);
  // fm_demod = gr::analog::quadrature_demod_cf::make(fm_demod_gain);

  // double symbol_decim = 1;

  // for (int i = 0; i < samples_per_symbol; i++) {
  //   sym_taps.push_back(1.0 / samples_per_symbol);
  // }

  // sym_filter    =  gr::filter::fir_filter_fff::make(symbol_decim, sym_taps);
  tune_queue    = gr::msg_queue::make(2);
  traffic_queue = gr::msg_queue::make(2);
  rx_queue      = queue;
  const double l[] = { -2.0, 0.0, 2.0, 4.0 };
  std::vector<float> levels(l, l + sizeof(l) / sizeof(l[0]));
  // fsk4_demod = gr::op25_repeater::fsk4_demod_ff::make(tune_queue, system_channel_rate, symbol_rate);
  slicer     = gr::op25_repeater::fsk4_slicer_fb::make(levels);

  int udp_port               = 0;
  int verbosity              = 0;
  const char *wireshark_host = "127.0.0.1";
  bool do_imbe               = 0;
  bool idle_silence          = 0;
  bool do_output             = 0;
  bool do_msgq               = 1;
  bool do_audio_output       = 0;
  bool do_tdma               = 0;
  op25_frame_assembler = gr::op25_repeater::p25_frame_assembler::make(sys_num, wireshark_host, udp_port, verbosity, do_imbe, do_output, idle_silence, do_msgq, rx_queue, do_audio_output, do_tdma);

  // if (!qpsk_mod) {
  //   connect(self(),        0, prefilter,            0);
  //   connect(prefilter,     0, channel_lpf,          0);
  //   connect(channel_lpf,   0, arb_resampler,        0);
  //   connect(arb_resampler, 0, fm_demod,             0);
  //   connect(fm_demod,      0, baseband_amp,         0);
  //   connect(baseband_amp,  0, sym_filter,           0);
  //   connect(sym_filter,    0, fsk4_demod,           0);
  //   connect(fsk4_demod,    0, slicer,               0);
  //   connect(slicer,        0, op25_frame_assembler, 0);
  // } else {
  //   connect(self(),        0, prefilter,            0);
  //   connect(prefilter,     0, channel_lpf,          0);
  //   connect(channel_lpf,   0, arb_resampler,        0);
  //   connect(arb_resampler, 0, agc,                  0);
  //   connect(agc,           0, costas_clock,         0);
  //   connect(costas_clock,  0, diffdec,              0);
  //   connect(diffdec,       0, to_float,             0);
  //   connect(to_float,      0, rescale,              0);
  //   connect(rescale,       0, slicer,               0);
  //   connect(slicer,        0, op25_frame_assembler, 0);
  // }
    connect(self(),        0, s2ss_block,           0);
    for (int i=0; i < channels; ++i) {
      connect(s2ss_block, i, pf_channelizer, i);
    }
    connect(pf_channelizer,  0, agc,        0);
    //connect(pf_channelizer,  0, arb_resampler,        0);
    // // BOOST_LOG_TRIVIAL(info) << "Connecting.";
    // for (int i=1; i<channels; ++i) {
    //   BOOST_LOG_TRIVIAL(info) << i;
    //   connect(pf_channelizer, i , null_sinks[i-1], 0);
    // }
    // connect(arb_resampler, 0, agc,                  0);
    connect(agc,           0, costas_clock,         0);
    connect(costas_clock,  0, diffdec,              0);
    connect(diffdec,       0, to_float,             0);
    connect(to_float,      0, rescale,              0);
    connect(rescale,       0, slicer,               0);
    connect(slicer,        0, op25_frame_assembler, 0);

}

p25_trunking::~p25_trunking() {}

double p25_trunking::get_freq() {
  return chan_freq;
}

void p25_trunking::tune_offset(double f) {
  chan_freq = f;
  int offset_amount = (f - center_freq);
  // prefilter->set_center_freq(offset_amount);
  BOOST_LOG_TRIVIAL(info) << "P25 Trunking Retune requested to " << chan_freq;
}
