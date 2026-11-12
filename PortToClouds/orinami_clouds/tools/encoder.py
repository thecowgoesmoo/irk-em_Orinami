#!/usr/bin/env python3
#
# QPSK encoder: converts a firmware .bin into a .wav for the Mutable
# Instruments audio bootloader.
#
# Python 3 port of stm_audio_bootloader/qpsk/encoder.py and
# audio_stream_writer.py, copyright 2013 Emilie Gillet, MIT licence.
# Behaviour is unchanged; only the language version differs.

import argparse
import struct
import sys
import zlib

import numpy


class QpskEncoder(object):

  def __init__(
      self,
      sample_rate=48000,
      carrier_frequency=6000,
      bit_rate=12000,
      packet_size=256):
    period = sample_rate // carrier_frequency
    symbol_time = sample_rate // (bit_rate // 2)

    assert (symbol_time % period == 0) or (period % symbol_time == 0)
    assert (sample_rate % carrier_frequency) == 0
    assert (sample_rate % bit_rate) == 0

    self._sr = sample_rate
    self._br = bit_rate
    self._carrier_frequency = carrier_frequency
    self._sample_index = 0
    self._packet_size = packet_size

  @staticmethod
  def _upsample(x, factor):
    return numpy.tile(x.reshape(len(x), 1), (1, factor)).ravel()

  def _encode_qpsk(self, symbol_stream):
    ratio = self._sr // self._br * 2
    symbol_stream = numpy.array(symbol_stream, dtype=int)
    bitstream_even = 2 * self._upsample(symbol_stream % 2, ratio) - 1
    bitstream_odd = 2 * self._upsample(symbol_stream // 2, ratio) - 1
    return bitstream_even / numpy.sqrt(2.0), bitstream_odd / numpy.sqrt(2.0)

  def _modulate(self, q_mod, i_mod):
    num_samples = len(q_mod)
    t = (numpy.arange(0.0, num_samples) + self._sample_index) / self._sr
    self._sample_index += num_samples
    phase = 2 * numpy.pi * self._carrier_frequency * t
    return q_mod * numpy.sin(phase) + i_mod * numpy.cos(phase)

  def _encode(self, symbol_stream):
    return self._modulate(*self._encode_qpsk(symbol_stream))

  def _code_blank(self, duration):
    num_zeros = int(duration * self._br / 8) * 4
    return self._encode(numpy.zeros(num_zeros, dtype=int))

  def _code_packet(self, data):
    assert len(data) <= self._packet_size
    if len(data) != self._packet_size:
      data = data + b'\xff' * (self._packet_size - len(data))

    crc = zlib.crc32(data) & 0xffffffff

    # Preamble: zeros for the PLL, then alignment and sync patterns.
    preamble = [0] * 8 + [0x99] * 4 + [0xcc] * 4
    crc_bytes = [crc >> 24, (crc >> 16) & 0xff, (crc >> 8) & 0xff, crc & 0xff]
    all_bytes = preamble + list(data) + crc_bytes

    symbol_stream = []
    for byte in all_bytes:
      symbol_stream.append((byte >> 6) & 0x3)
      symbol_stream.append((byte >> 4) & 0x3)
      symbol_stream.append((byte >> 2) & 0x3)
      symbol_stream.append((byte >> 0) & 0x3)

    return self._encode(symbol_stream)

  def code_intro(self):
    yield numpy.zeros(1 * self._sr)
    yield self._code_blank(1.0)

  def code_outro(self, duration=1.0):
    yield self._code_blank(duration)

  def code(self, data, page_size=1024, blank_duration=0.06):
    if len(data) % page_size != 0:
      tail = page_size - (len(data) % page_size)
      data += b'\xff' * tail

    offset = 0
    remaining_bytes = len(data)
    num_packets_written = 0
    while remaining_bytes:
      size = min(remaining_bytes, self._packet_size)
      yield self._code_packet(data[offset:offset + size])
      num_packets_written += 1
      if num_packets_written == page_size // self._packet_size:
        yield self._code_blank(blank_duration)
        num_packets_written = 0
      remaining_bytes -= size
      offset += size


STM32F4_SECTOR_BASE_ADDRESS = [
  0x08000000, 0x08004000, 0x08008000, 0x0800C000,
  0x08010000, 0x08020000, 0x08040000, 0x08060000,
  0x08080000, 0x080A0000, 0x080C0000, 0x080E0000
]

PAGE_SIZE = { 'stm32f1': 1024, 'stm32f3': 2048 }
PAUSE = { 'stm32f1': 0.06, 'stm32f3': 0.15 }
STM32F4_BLOCK_SIZE = 16384
STM32F4_APPLICATION_START = 0x08008000


def write_wav(file_name, sample_rate, blocks):
  signal = numpy.concatenate(blocks)
  signal = numpy.clip(signal, -1.0, 1.0)
  samples = numpy.array(signal * 32767, dtype='<i2')
  data = samples.tobytes()
  with open(file_name, 'wb') as f:
    f.write(b'RIFF')
    f.write(struct.pack('<L', 4 + 8 + 16 + 8 + len(data)))
    f.write(b'WAVEfmt ')
    f.write(struct.pack(
        '<LHHLLHH', 16, 1, 1, sample_rate, sample_rate * 2, 2, 16))
    f.write(b'data')
    f.write(struct.pack('<L', len(data)))
    f.write(data)


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('-s', '--sample_rate', type=int, default=48000)
  parser.add_argument('-c', '--carrier_frequency', type=int, default=6000)
  parser.add_argument('-b', '--baud_rate', type=int, default=12000)
  parser.add_argument('-p', '--packet_size', type=int, default=256)
  parser.add_argument('-o', '--output_file', default=None)
  parser.add_argument('-t', '--target', default='stm32f1',
                      choices=['stm32f1', 'stm32f3', 'stm32f4'])
  parser.add_argument('bin_file')
  options = parser.parse_args()

  with open(options.bin_file, 'rb') as f:
    data = f.read()

  output_file = options.output_file
  if not output_file:
    if '.bin' in options.bin_file:
      output_file = options.bin_file.replace('.bin', '.wav')
    else:
      output_file = options.bin_file + '.wav'

  encoder = QpskEncoder(
      options.sample_rate,
      options.carrier_frequency,
      options.baud_rate,
      options.packet_size)

  blocks = list(encoder.code_intro())
  blank_duration = 1.0
  if options.target in PAGE_SIZE:
    blocks += [b for b in encoder.code(
        data, PAGE_SIZE[options.target], PAUSE[options.target]) if len(b)]
  else:
    for x in range(0, len(data), STM32F4_BLOCK_SIZE):
      address = STM32F4_APPLICATION_START + x
      chunk = data[x:x + STM32F4_BLOCK_SIZE]
      # Leave time for a flash sector erase at each sector boundary.
      pause = 3.5 if address in STM32F4_SECTOR_BASE_ADDRESS else 0.2
      blocks += [b for b in encoder.code(chunk, STM32F4_BLOCK_SIZE, pause)
                 if len(b)]
    blank_duration = 5.0
  blocks += list(encoder.code_outro(blank_duration))

  write_wav(output_file, options.sample_rate, blocks)
  print('%s: %.1f s' % (
      output_file, sum(len(b) for b in blocks) / options.sample_rate))


if __name__ == '__main__':
  sys.exit(main())
