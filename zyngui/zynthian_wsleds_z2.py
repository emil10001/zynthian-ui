#!/usr/bin/python3
# -*- coding: utf-8 -*-
#******************************************************************************
# ZYNTHIAN PROJECT: Zynthian GUI
#
# Zynthian WSLeds Class for Z2
#
# Copyright (C) 2015-2023 Fernando Moyano <jofemodo@zynthian.org>
#
#******************************************************************************
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
#******************************************************************************

# Zynthian specific modules
from zyngui.zynthian_wsleds_base import zynthian_wsleds_base
from zyngui import zynthian_gui_config

# ---------------------------------------------------------------------------
# Zynthian WSLeds class for Z2
# ---------------------------------------------------------------------------
class zynthian_wsleds_z2(zynthian_wsleds_base):
	
	def __init__(self, zyngui):
		super().__init__(zyngui)
		if zynthian_gui_config.wiring_layout == "Z2_V1":
			# LEDS with PWM1 (pin 13, channel 1)
			self.dma = 10
			self.pin = 13
			self.chan = 1
			self.num_leds = 25
		elif zynthian_gui_config.wiring_layout in ("Z2_V2", "Z2_V3"):
			# LEDS with SPI0 (pin 10, channel 0)
			self.dma = 10
			self.pin = 10
			self.chan = 0
			self.num_leds = 25


	def update_wsleds(self):
		curscreen = self.zyngui.current_screen
		curscreen_obj = self.zyngui.get_current_screen_obj()

		# Menu
		if self.zyngui.is_current_screen_menu():
			self.wsleds.setPixelColor(0, self.wscolor_active)
		elif curscreen == "admin":
			self.wsleds.setPixelColor(0, self.wscolor_active2)
		else:
			self.wsleds.setPixelColor(0, self.wscolor_default)

		# Active Chain
		if self.zyngui.alt_mode:
			wscolor_light = self.wscolor_alt
			offset = 5
		else:
			wscolor_light = self.wscolor_default
			offset = 0
		# => Light non-empty chains
		for i, chain_id in enumerate(["01", "02", "03", "04", "05", "main"]):
			if self.zyngui.chain_manager.get_chain(chain_id) is None:
				self.wsleds.setPixelColor(i + 1, self.wscolor_off)
			else:
				if self.zyngui.chain_manager.active_chain_id == chain_id:
					# => Light active chain
					if curscreen == "control":
						self.wsleds.setPixelColor(i + 1, self.wscolor_active)
					else:
						if self.zyngui.chain_manager.get_processor_count(chain_id):
							self.blink(i + 1, self.wscolor_active)
						else:
							self.blink(i + 1, self.wscolor_active2)
				else:
					self.wsleds.setPixelColor(i + 1, wscolor_light)

		# MODE button => MIDI LEARN
		if self.zyngui.state_manager.get_midi_learn_zctrl() or curscreen == "zs3":
			self.wsleds.setPixelColor(7, self.wscolor_yellow)
		elif self.zyngui.state_manager.midi_learn_zctrl:
			self.wsleds.setPixelColor(7, self.wscolor_active)
		else:
			self.wsleds.setPixelColor(7, self.wscolor_default)

		# Zynpad screen:
		if curscreen == "zynpad":
			self.wsleds.setPixelColor(8, self.wscolor_active)
		else:
			self.wsleds.setPixelColor(8, self.wscolor_default)

		# Pattern Editor/Arranger screen:
		if curscreen == "pattern_editor":
			self.wsleds.setPixelColor(9, self.wscolor_active)
		elif curscreen == "arranger":
			self.wsleds.setPixelColor(9, self.wscolor_active2)
		else:
			self.wsleds.setPixelColor(9, self.wscolor_default)

		# Control / Preset Screen:
		if curscreen in ("control", "audio_player"):
			self.wsleds.setPixelColor(10, self.wscolor_active)
		elif curscreen in ("preset", "bank"):
			if self.zyngui.current_processor.get_show_fav_presets():
				self.blink(10, self.wscolor_active2)
			else:
				self.wsleds.setPixelColor(10, self.wscolor_active2)
		else:
			self.wsleds.setPixelColor(10, self.wscolor_default)

		# ZS3/Snapshot screen:
		if curscreen == "zs3":
			self.wsleds.setPixelColor(11, self.wscolor_active)
		elif curscreen == "snapshot":
			self.wsleds.setPixelColor(11, self.wscolor_active2)
		else:
			self.wsleds.setPixelColor(11, self.wscolor_default)

		# ???:
		self.wsleds.setPixelColor(12, self.wscolor_default)

		# ALT button:
		if self.zyngui.alt_mode:
			self.wsleds.setPixelColor(13, self.wscolor_alt)
		else:
			self.wsleds.setPixelColor(13, self.wscolor_default)

		wsleds = [14, 17, 15]
		update_wsleds_func = getattr(curscreen_obj, "update_wsleds", None)
		if callable(update_wsleds_func):
			update_wsleds_func(wsleds)
		else:
			if self.zyngui.alt_mode:
				self.zyngui.screens["midi_recorder"].update_wsleds(wsleds)
			else:
				# REC Button
				if self.zyngui.state_manager.status_audio_recorder:
					self.wsleds.setPixelColor(wsleds[0], self.wscolor_red)
				else:
					self.wsleds.setPixelColor(wsleds[0], self.wscolor_default)
				# STOP button
				self.wsleds.setPixelColor(wsleds[1], self.wscolor_default)
				# PLAY button:
				if self.zyngui.state_manager.status_audio_player:
					self.wsleds.setPixelColor(wsleds[2], self.wscolor_green)
				else:
					self.wsleds.setPixelColor(wsleds[2], self.wscolor_default)

		# Tempo Screen
		if curscreen == "tempo":
			self.wsleds.setPixelColor(16, self.wscolor_active)
		elif self.zyngui.state_manager.zynseq.libseq.isMetronomeEnabled():
			self.blink(16, self.wscolor_active)
		else:
			self.wsleds.setPixelColor(16, self.wscolor_default)

		# Select/Yes button
		self.wsleds.setPixelColor(20, self.wscolor_green)

		# Back/No button
		self.wsleds.setPixelColor(18, self.wscolor_red)

		# Arrow buttons (Up, Left, Bottom, Right)
		self.wsleds.setPixelColor(19, self.wscolor_yellow)
		self.wsleds.setPixelColor(21, self.wscolor_yellow)
		self.wsleds.setPixelColor(22, self.wscolor_yellow)
		self.wsleds.setPixelColor(23, self.wscolor_yellow)

		# Audio Mixer / ALSA Mixer
		if curscreen == "audio_mixer":
			self.wsleds.setPixelColor(24, self.wscolor_active)
		elif curscreen == "alsa_mixer":
			self.wsleds.setPixelColor(24, self.wscolor_active2)
		else:
			self.wsleds.setPixelColor(24, self.wscolor_default)

		try:
			self.zyngui.screens[curscreen].update_wsleds()
		except:
			pass

#------------------------------------------------------------------------------
