#!/usr/bin/python3
# -*- coding: utf-8 -*-
# ******************************************************************************
# ZYNTHIAN PROJECT: Zynthian Control Device Driver
#
# Zynthian Control Device Driver for "Akai APC mini mk2"
#
# Copyright (C) 2023,2024 Oscar Aceña <oscaracena@gmail.com>
#
# ******************************************************************************
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation; either version 2 of
# the License, or any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# For a full copy of the GNU General Public License see the LICENSE.txt file.
#
# ******************************************************************************

import time
import signal
import jack
import logging
from bisect import bisect
from copy import deepcopy
from functools import partial
import multiprocessing as mp
from threading import Thread, RLock, Event

from zynlibs.zynseq import zynseq
from zyncoder.zyncore import lib_zyncore
from zyngine.zynthian_signal_manager import zynsigman
from zyngine.zynthian_engine_audioplayer import zynthian_engine_audioplayer

from .zynthian_ctrldev_base import (
    zynthian_ctrldev_zynmixer, zynthian_ctrldev_zynpad
)
from .zynthian_ctrldev_base_extended import (
    RunTimer, KnobSpeedControl, ButtonTimer, CONST
)
from .zynthian_ctrldev_base_ui import ModeHandlerBase

from .zynthian_ctrldev_akai_apc_generic import DeviceConfig, zynthian_ctrldev_akai_apc

# FIXME: these defines should be taken from where they are defined (zynseq.h)
MAX_STUTTER_COUNT                    = 32
MAX_STUTTER_DURATION                 = 96

# MIDI channel events (first 4 bits), next 4 bits is the channel!
EV_NOTE_ON                           = 0x09
EV_NOTE_OFF                          = 0x08
EV_CC                                = 0x0B

# MIDI system events (first 8 bits)
EV_SYSEX                             = 0xF0
EV_CLOCK                             = 0xF8
EV_CONTINUE                          = 0xFB

dev_configs = DeviceConfig (
    dev_ids = ["APC mini mk2 IN 1", "APC mini mk2 IN 2",
                "APC mini mk2 1", "APC mini mk2 2",
                "APC mini mk2 MIDI 1", "APC mini mk2 MIDI 2"],
    # APC mini buttons
    BTN_SHIFT                 = 0x7A,
    BTN_STOP_ALL_CLIPS        = 0x77,
    BTN_PLAY                  = 0xFF,
    BTN_RECORD                = 0xFE,
    BTN_TRACK_1               = 0x68,
    BTN_TRACK_2               = 0x69,
    BTN_TRACK_3               = 0x6A,
    BTN_TRACK_4               = 0x6B,
    BTN_TRACK_5               = 0x64,
    BTN_TRACK_6               = 0x65,
    BTN_TRACK_7               = 0x66,
    BTN_TRACK_8               = 0x67,
    BTN_UP                    = 0x68,
    BTN_DOWN                  = 0x69,
    BTN_LEFT                  = 0x6A,
    BTN_RIGHT                 = 0x6B,
    BTN_KNOB_CTRL_VOLUME      = 0x64,
    BTN_KNOB_CTRL_PAN         = 0x65,
    BTN_KNOB_CTRL_SEND        = 0x66,
    BTN_KNOB_CTRL_DEVICE      = 0x67,
    BTN_SOFT_KEY_START        = 0x70,
    BTN_SOFT_KEY_END          = 0x74,
    BTN_SOFT_KEY_CLIP_STOP    = 0x70,
    BTN_SOFT_KEY_SOLO         = 0x71,
    BTN_SOFT_KEY_MUTE         = 0x72,
    BTN_SOFT_KEY_REC_ARM      = 0x73,
    BTN_KNOB_1                = 0x70,
    BTN_KNOB_2                = 0x71,
    BTN_KNOB_3                = 0x72,
    BTN_KNOB_4                = 0x73,
    BTN_SOFT_KEY_SELECT       = 0x74,
    BTN_PAD_START             = 0x00,
    BTN_PAD_END               = 0x3F,
    BTN_PAD_5                 = 0x04,
    BTN_PAD_6                 = 0x05,
    BTN_PAD_7                 = 0x06,
    BTN_PAD_8                 = 0x07,
    BTN_PAD_13                = 0x0C,
    BTN_PAD_14                = 0x0D,
    BTN_PAD_15                = 0x0E,
    BTN_PAD_16                = 0x0F,
    BTN_PAD_21                = 0x14,
    BTN_PAD_23                = 0x16,
    BTN_PAD_24                = 0x17,
    BTN_PAD_29                = 0x1C,
    BTN_PAD_30                = 0x1D,
    BTN_PAD_31                = 0x1E,
    BTN_PAD_32                = 0x1F,
    BTN_PAD_37                = 0x24,
    BTN_PAD_38                = 0x25,
    BTN_PAD_39                = 0x26,
    BTN_PAD_40                = 0x27,
    BTN_PAD_LEFT              = 0x04,
    BTN_PAD_DOWN              = 0x05,
    BTN_PAD_RIGHT             = 0x06,
    BTN_F4                    = 0x07,
    BTN_BACK_NO               = 0x0C,
    BTN_PAD_UP                = 0x0D,
    BTN_SEL_YES               = 0x0E,
    BTN_F3                    = 0x0F,
    BTN_PAD_RECORD            = 0x14,
    BTN_PAD_PLAY              = 0x16,
    BTN_F2                    = 0x17,
    BTN_ALT                   = 0x1C,
    BTN_METRONOME             = 0x1D,
    BTN_PAD_STEP              = 0x1E,
    BTN_F1                    = 0x1F,
    BTN_OPT_ADMIN             = 0x24,
    BTN_MIX_LEVEL             = 0x25,
    BTN_CTRL_PRESET           = 0x26,
    BTN_ZS3_SHOT              = 0x27,
    KNOB_1                    = 0x30,
    KNOB_2                    = 0x31,
    KNOB_3                    = 0x32,
    KNOB_4                    = 0x33,
    KNOB_5                    = 0x34,
    KNOB_6                    = 0x35,
    KNOB_7                    = 0x36,
    KNOB_8                    = 0x37,
    KNOB_LAYER                = 0x30,
    KNOB_SNAPSHOT             = 0x31,
    KNOB_BACK                 = 0x34,
    KNOB_SELECT               = 0x35,
)


# --------------------------------------------------------------------------
# 'Akai APC mini mk2' device controller class
# --------------------------------------------------------------------------
class zynthian_ctrldev_akai_apc_mini_mk2: zynthian_ctrldev_akai_apc(zynthian_ctrldev_zynmixer, zynthian_ctrldev_zynpad, DeviceConfig = dev_configs)
