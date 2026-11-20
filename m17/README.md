# M17 接收

固件里的 M17 接收（只收）：HR_C7000 以 8 kHz 录下鉴频器输出（和 NXDN 一样），软件解调、帧处理、Codec 2 3200
解码，经提示音 FIFO 播放。全部是依据 M17 规范和 Codec 2 参考实现的算法自行实现的整数代码，没有第三方代码。

| 文件 | 内容 |
|---|---|
| `src/services/m17_demod.c` | 4FSK 软件解调：8 kHz 上按分数时间三次插值取符号，同步字最小二乘拟合时序、增益、直流 |
| `src/services/m17_core.c` | 帧：同步、去扰码、QPP 解交织、Viterbi（P1/P2 打孔）、CRC、LICH（Golay 24,12）拼 LSF、呼号 |
| `src/services/c2dec.c` | Codec 2 3200 定点解码，算法见 [codec2-3200.md](codec2-3200.md) |
| `src/services/m17_radio.c` | 录音 → 解调 → 帧 → 解码 → 播放；状态、解码器、任务栈都在 M17 覆盖区 RAM |
| `src/services/m17_bench.c` | 调试版：在收音机上给 Codec 2 解码计时 |

## PC 上的测试（`C:/msys64/ucrt64/bin/gcc`）

```sh
gcc -O2 -Isrc/services -o m17/out/m17_demod_test.exe m17/c/m17_demod_test.c src/services/m17_demod.c src/services/m17_core.c
gcc -O2 -Isrc/services -o m17/out/m17_core_test.exe m17/c/m17_core_test.c src/services/m17_core.c
gcc -O2 -Isrc/services -o m17/out/c2dec_run.exe m17/c/c2dec_run.c src/services/c2dec.c
```

- `m17_demod_test in.raw [out.c2]`：8 kHz 16 位录音 → 解调 → 帧。`data/real_stream.raw`（收音机录的流，中途开始）、
  `data/call_start.raw`（一次发射的开头，前后共 1.5 s）。
- `py/m17_sim.py frames.txt out.raw [--snr --dc --ppm]`：按帧文件仿真 8 kHz 鉴频器录音（40 ms 标准前导码、RRC α=0.5、
  噪声、直流、时钟偏差）。
- `m17_core_test in.dibits [out.c2] [--preamble] [--gap FROM N] [--flip K]`：直接喂 dibit，可模拟擦除和误码。
- `c2dec_run in.c2 out.raw`；`py/c2_3200_model.py`：Codec 2 3200 的浮点模型（与参考解码器核对过）；
  `py/c2_check.py ref.raw ours.raw`：比较两份解码输出。

## 测试信号（热点）

- `data/m17_tone1k.txt`：纯单音（约 1 kHz，33 dB），由 `py/c2_tone.py` 搜索出的 Codec 2 帧 `data/tone1k.c2`
  经 `py/m17_frames.py` 打包；`data/m17_zh.txt`：thchs30 中文语音 10 段（96.6 s），参考库编码后打包。
- 热点上用 MMDVM 的 M17 模式逐帧发送（pistar-signal-gen 的发射器加了 M17 帧类型），见 ai repo 的
  `OpenDM32/notes/dm32-m17-2026-11-19.md`。热点开始发射时会丢掉文件开头的 LSF，所以测试里呼号来自 LICH。
- `py/m17_trials.py FILE N`：连发 N 次，每次后读收音机的计数；`py/m17_monitor.py --port COM22 [--tap] [--raw]`：
  读计数、最后的 dibit、通话开头前后的录音（调试版固件）；`py/af_fit.py`：录音与已知符号对齐，求增益、频偏。
- `py/codec2_radio_bench.py in.c2 --port COM22 [--profile]`：收音机上 Codec 2 解码耗时（调试版）。
