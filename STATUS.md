# 暂定发行版状态与串口数据读取指南

## 当前状态

当前分支可作为 SLE 测距与 IMU 数据采集的暂定发行版。Server 通过串口输出两类 CSV 记录：

- `IMU`：Client 采集并通过 SLE 发送的九轴数据，目标有效采样率为 20 Hz。
- `DIST`：Server 计算得到的 SLE 测距结果，当前测量频率约为 2 Hz。

上电阶段可能出现 bootloader、时钟切换或波特率切换产生的乱码。串口读取程序不得尝试从乱码中截取
`IMU,` 或 `DIST,` 后面的局部内容；整行不合法时直接丢弃。

## IMU 记录

IMU 行必须严格以 `IMU,` 开头，并且按逗号分隔后必须正好有 **13 个字段**：

```text
IMU,batch_sequence,sample_sequence,timestamp_us,ax,ay,az,gx,gy,gz,mx,my,mz
```

| 序号 | 字段 | 类型 | 单位及含义 |
|---:|---|---|---|
| 1 | `IMU` | 固定字符串 | 记录类型，必须完全等于 `IMU` |
| 2 | `batch_sequence` | 无符号整数 | IMU 批次序号 |
| 3 | `sample_sequence` | 无符号整数 | IMU 样本序号，可用于检查丢样 |
| 4 | `timestamp_us` | 无符号整数 | Client 从启动开始计时的样本时间戳，单位为微秒 |
| 5 | `ax` | 有限小数 | X 轴加速度，单位为 g |
| 6 | `ay` | 有限小数 | Y 轴加速度，单位为 g |
| 7 | `az` | 有限小数 | Z 轴加速度，单位为 g |
| 8 | `gx` | 有限小数 | X 轴角速度，单位为度每秒 |
| 9 | `gy` | 有限小数 | Y 轴角速度，单位为度每秒 |
| 10 | `gz` | 有限小数 | Z 轴角速度，单位为度每秒 |
| 11 | `mx` | 有限小数 | X 轴磁力计原始值，尚未进行板级校准 |
| 12 | `my` | 有限小数 | Y 轴磁力计原始值，尚未进行板级校准 |
| 13 | `mz` | 有限小数 | Z 轴磁力计原始值，尚未进行板级校准 |

示例：

```text
IMU,29,116,6047455,0.056641,0.814331,0.549072,74.427,92.107,32.977,-394.000,-1880.000,-219.000
```

## DIST 记录

DIST 行必须严格以 `DIST,` 开头，并且按逗号分隔后必须正好有 **6 个字段**：

```text
DIST,distance_sequence,measurement_timestamp_us,result_timestamp_us,distance_m,calculation_time_ms
```

| 序号 | 字段 | 类型 | 单位及含义 |
|---:|---|---|---|
| 1 | `DIST` | 固定字符串 | 记录类型，必须完全等于 `DIST` |
| 2 | `distance_sequence` | 无符号整数 | 测距结果序号，可用于检查丢失 |
| 3 | `measurement_timestamp_us` | 无符号整数 | Server 收到本次 IQ 报告的时间，单位为微秒 |
| 4 | `result_timestamp_us` | 无符号整数 | Server 完成本次测距计算的时间，单位为微秒 |
| 5 | `distance_m` | 有限小数 | 距离，单位为米；负数表示本次结果无效 |
| 6 | `calculation_time_ms` | 无符号整数 | 测距计算耗时，单位为毫秒 |

示例：

```text
DIST,8,19173766,19525127,0.620,210
```

## 必须遵守的解析规则

1. 打开串口后立即清空已有输入缓冲区，避免把上电乱码或旧数据交给实时算法。
2. 按 `\n` 读取完整行，只删除行尾的 `\r\n`，不得删除或拼接字段内部字符。
3. 只接受严格以 `IMU,` 或 `DIST,` 开头的行。前面有任何乱码、空格或其他前缀都视为无效。
4. IMU 必须正好有 13 个字段；DIST 必须正好有 6 个字段。
5. 所有字段必须完整转换。转换函数必须消费字段的全部字符，不能接受 `123abc`、`-742 .000`、空字段、
   `NaN` 或 `Inf` 等只转换了数值前半段或不是有限数值的内容。
6. 任一字段校验或转换失败时，丢弃整行。不要保留部分字段，也不要用上一条记录补齐。
7. 丢弃无效行不应阻塞后续读取；下一条完整合法行仍应正常进入处理流程。
8. PDR 使用 `timestamp_us` 计算相邻 IMU 样本的相对时间；测距处理使用 DIST 自己的序号和时间戳。
9. `distance_m < 0` 是格式合法但测距结果无效的记录：可以用于统计链路状态，但不得送入距离/角度计算。

## Python/pyserial 参考脚本

安装依赖：

```shell
python3 -m pip install pyserial
```

下面的脚本要求显式提供串口和波特率。例如：

```shell
python3 read_telemetry.py --port /dev/cu.usbserial-0001 --baud 115200
```

```python
#!/usr/bin/env python3
import argparse
import math
import re
from dataclasses import dataclass

import serial


UINT_RE = re.compile(r"[0-9]+")
FLOAT_RE = re.compile(r"[+-]?[0-9]+\.[0-9]+")


@dataclass(frozen=True)
class ImuRecord:
    batch_sequence: int
    sample_sequence: int
    timestamp_us: int
    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float
    mx: float
    my: float
    mz: float


@dataclass(frozen=True)
class DistanceRecord:
    distance_sequence: int
    measurement_timestamp_us: int
    result_timestamp_us: int
    distance_m: float
    calculation_time_ms: int


def parse_uint(token: str) -> int:
    if UINT_RE.fullmatch(token) is None:
        raise ValueError("invalid unsigned integer")
    return int(token, 10)


def parse_float(token: str) -> float:
    # fullmatch 保证不会只转换诸如 "-742 .000" 或 "1.2abc" 的数值前半段。
    if FLOAT_RE.fullmatch(token) is None:
        raise ValueError("invalid decimal")
    value = float(token)
    if not math.isfinite(value):
        raise ValueError("non-finite decimal")
    return value


def parse_line(line: str) -> ImuRecord | DistanceRecord | None:
    try:
        if line.startswith("IMU,"):
            fields = line.split(",")
            if len(fields) != 13 or fields[0] != "IMU":
                return None
            return ImuRecord(
                batch_sequence=parse_uint(fields[1]),
                sample_sequence=parse_uint(fields[2]),
                timestamp_us=parse_uint(fields[3]),
                ax=parse_float(fields[4]),
                ay=parse_float(fields[5]),
                az=parse_float(fields[6]),
                gx=parse_float(fields[7]),
                gy=parse_float(fields[8]),
                gz=parse_float(fields[9]),
                mx=parse_float(fields[10]),
                my=parse_float(fields[11]),
                mz=parse_float(fields[12]),
            )

        if line.startswith("DIST,"):
            fields = line.split(",")
            if len(fields) != 6 or fields[0] != "DIST":
                return None
            return DistanceRecord(
                distance_sequence=parse_uint(fields[1]),
                measurement_timestamp_us=parse_uint(fields[2]),
                result_timestamp_us=parse_uint(fields[3]),
                distance_m=parse_float(fields[4]),
                calculation_time_ms=parse_uint(fields[5]),
            )
    except (ValueError, OverflowError):
        return None

    return None


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True, help="串口设备路径")
    parser.add_argument("--baud", required=True, type=int, help="固件当前使用的波特率")
    args = parser.parse_args()

    with serial.Serial(args.port, args.baud, timeout=1) as port:
        # 丢弃程序启动前已经积存在驱动缓冲区中的上电乱码和旧数据。
        port.reset_input_buffer()

        while True:
            raw_line = port.readline()
            if not raw_line.endswith(b"\n"):
                # 超时得到的不完整行必须丢弃。
                continue
            try:
                line = raw_line.rstrip(b"\r\n").decode("ascii", errors="strict")
            except UnicodeDecodeError:
                continue

            record = parse_line(line)
            if record is None:
                continue

            if isinstance(record, ImuRecord):
                # 在这里把 record 送入实时 PDR。
                print(record)
            elif record.distance_m >= 0.0:
                # 在这里把有效距离送入实时角度计算。
                print(record)


if __name__ == "__main__":
    main()
```

生产程序可以将两个 `print(record)` 替换为线程安全队列或回调。串口读取循环应保持轻量，PDR 和角度计算不应
阻塞串口读取，以免操作系统串口缓冲区积压。
