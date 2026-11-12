// Feeds a firmware-update WAV through the bootloader's own demodulator and
// packet decoder (same loop as clouds/bootloader/bootloader.cc) and compares
// what would be written to flash with the .bin. Desktop tool; build from the
// root of the eurorack repository:
//
//   g++ -std=c++11 -O2 -I. -DTEST orinami_clouds/tools/boot_verify.cc \
//       stm_audio_bootloader/qpsk/demodulator.cc \
//       stm_audio_bootloader/qpsk/packet_decoder.cc -o boot_verify
//   ./boot_verify firmware.wav firmware.bin [gain] [seconds_busy_per_block]
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include "stm_audio_bootloader/qpsk/packet_decoder.h"
#include "stm_audio_bootloader/qpsk/demodulator.h"
using namespace stm_audio_bootloader;
const double kSampleRate = 48000.0, kModulationRate = 6000.0, kBitRate = 12000.0;
const uint32_t kBlockSize = 16384;
const uint16_t kPacketsPerBlock = kBlockSize / kPacketSize;
int main(int argc, char** argv) {
  FILE* f = fopen(argv[1], "rb"); fseek(f, 44, SEEK_SET);
  std::vector<int16_t> wav; int16_t s; while (fread(&s, 2, 1, f) == 1) wav.push_back(s); fclose(f);
  f = fopen(argv[2], "rb"); std::vector<uint8_t> bin; int c; while ((c = fgetc(f)) != EOF) bin.push_back(c); fclose(f);
  while (bin.size() % kBlockSize) bin.push_back(0xff);
  float gain = argc > 3 ? atof(argv[3]) : 1.0f; float busy = argc > 4 ? atof(argv[4]) : 0.4f;

  PacketDecoder decoder; Demodulator demodulator;
  decoder.Init(20000);
  demodulator.Init(kModulationRate / kSampleRate * 4294967296.0, kSampleRate / kModulationRate, 2.0 * kSampleRate / kBitRate);
  demodulator.SyncCarrier(true); decoder.Reset();
  std::vector<uint8_t> flash; static uint8_t rx_buffer[kBlockSize]; uint16_t packet_index = 0;
  size_t pos = 8000; bool done = false, error = false;
  while (!done && !error && pos < wav.size()) {
    for (int k = 0; k < 32 && pos < wav.size(); ++k) { int32_t sample = ((int16_t)(wav[pos++] * gain) >> 4) + 2048; demodulator.PushSample(sample); }
    if (demodulator.state() == DEMODULATOR_STATE_OVERFLOW) { error = true; break; }
    demodulator.ProcessAtLeast(32);
    while (demodulator.available() && !error && !done) {
      uint8_t symbol = demodulator.NextSymbol();
      PacketDecoderState state = decoder.ProcessSymbol(symbol);
      switch (state) {
        case PACKET_DECODER_STATE_OK:
          memcpy(rx_buffer + (packet_index % kPacketsPerBlock) * kPacketSize, decoder.packet_data(), kPacketSize);
          ++packet_index;
          if ((packet_index % kPacketsPerBlock) == 0) { flash.insert(flash.end(), rx_buffer, rx_buffer + kBlockSize); /* the real bootloader is busy erasing and programming flash here */ pos += (size_t)(busy * 48000); decoder.Reset(); demodulator.SyncCarrier(false); }
          else { decoder.Reset(); demodulator.SyncDecision(); }
          break;
        case PACKET_DECODER_STATE_ERROR_SYNC: case PACKET_DECODER_STATE_ERROR_CRC: error = true; break;
        case PACKET_DECODER_STATE_END_OF_TRANSMISSION: done = true; break;
        default: break;
      }
    }
  }
  bool same = flash.size() == bin.size() && !memcmp(flash.data(), bin.data(), bin.size());
  printf("packets %d, error %d, end-of-transmission %d, decoded %zu bytes, bin (padded) %zu bytes, identical: %s\n", packet_index, error, done, flash.size(), bin.size(), same ? "YES" : "NO");
  return same ? 0 : 1;
}
