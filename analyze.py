#!/usr/bin/env python3
"""Анализ дампа логического анализатора (sigrok CSV, 2 канала).
Цель: определить тип протокола (I2C/SMBus, UART, single-wire) по таймингам переходов."""
import sys
import numpy as np

CSV = "зарядка_батарей_dji_hub.csv"

# Парсим: пропускаем строки заголовка (';') и строку с именами колонок.
# Формат данных: '<systime>,<time_s>,<ch0>,<ch1>
times, ch0, ch1 = [], [], []
with open(CSV, "r") as f:
    for line in f:
        if not line or line[0] in (";",) or line.startswith("SystemTime"):
            continue
        parts = line.rstrip("\r\n").split(",")
        if len(parts) != 4:
            continue
        try:
            t = float(parts[1])
            c0 = int(parts[2])
            c1 = int(parts[3])
        except ValueError:
            continue
        times.append(t)
        ch0.append(c0)
        ch1.append(c1)

t = np.array(times)
c0 = np.array(ch0, dtype=np.int8)
c1 = np.array(ch1, dtype=np.int8)
print(f"Загружено переходов: {len(t)}")
print(f"Длительность записи: {t[-1]-t[0]:.6f} с")
print()

def channel_stats(name, c):
    # переходы (где значение меняется)
    trans = np.flatnonzero(np.diff(c) != 0) + 1
    n_trans = len(trans)
    # доля времени в 1
    # интервалы между переходами этого канала
    if n_trans < 2:
        print(f"{name}: переходов={n_trans} (почти статичен, уровень={c[0]})")
        return
    tt = t[trans]
    dwell = np.diff(tt)  # время между переходами этого канала
    print(f"{name}: переходов={n_trans}")
    print(f"   мин. интервал между переходами: {dwell.min()*1e9:.0f} нс  ({1/dwell.min()/1e3:.1f} kHz если это полупериод)")
    # гистограмма «ширин импульсов» — ключ к битрейту
    # округлим до нс и посмотрим топ частых длительностей
    ns = np.round(dwell*1e9).astype(np.int64)
    vals, cnts = np.unique(ns, return_counts=True)
    order = np.argsort(-cnts)[:12]
    print(f"   топ длительностей импульсов (нс : сколько раз):")
    for i in order:
        print(f"      {vals[i]:>8d} ns  x{cnts[i]}")
    return tt, dwell

print("=== Channel 0 ===")
channel_stats("CH0", c0)
print()
print("=== Channel 1 ===")
channel_stats("CH1", c1)
print()

# Кто из каналов «clock-подобный» (регулярные равные импульсы) vs «data»?
# Активность во времени: разобьём на 1с-бины и посчитаем переходы.
print("=== Активность по секундам (переходы CH0 / CH1) ===")
edges = np.arange(0, t[-1]+1, 1.0)
d0 = np.flatnonzero(np.diff(c0) != 0)+1
d1 = np.flatnonzero(np.diff(c1) != 0)+1
h0,_ = np.histogram(t[d0], bins=edges)
h1,_ = np.histogram(t[d1], bins=edges)
for i in range(len(h0)):
    if h0[i] or h1[i]:
        bar0 = "#"*min(40,h0[i]//50)
        print(f"  t={i:>2}-{i+1:>2}s  CH0={h0[i]:>6}  CH1={h1[i]:>6}  {bar0}")
