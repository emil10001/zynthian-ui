#!/usr/bin/python3
# -*- coding: utf-8 -*-
# ******************************************************************************
# ZYNTHIAN PROJECT: Zynthian GUI
# 
# Zynthian GUI Admin Class
# 
# Copyright (C) 2015-2023 Fernando Moyano <jofemodo@zynthian.org>
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

from curses import A_HORIZONTAL
import os
import re
import signal
import logging
from time import sleep
from threading import Thread
from subprocess import check_output, Popen, PIPE, STDOUT

# Zynthian specific modules
import zynconf
import zynautoconnect
from zyncoder.zyncore import get_lib_zyncore
from zyngui import zynthian_gui_config
from zyngui.zynthian_gui_selector import zynthian_gui_selector
from zyngine import zynthian_gui_config

# -------------------------------------------------------------------------------
# Zynthian Admin GUI Class
# -------------------------------------------------------------------------------


class zynthian_gui_admin(zynthian_gui_selector):

	data_dir = os.environ.get('ZYNTHIAN_DATA_DIR', "/zynthian/zynthian-data")
	sys_dir = os.environ.get('ZYNTHIAN_SYS_DIR', "/zynthian/zynthian-sys")

	def __init__(self):
		self.commands = None
		self.thread = None
		self.child_pid = None
		self.last_action = None

		super().__init__('Action', True)

		self.state_manager = self.zyngui.state_manager

		if self.state_manager.allow_rbpi_headphones():
			self.default_rbpi_headphones()

	def fill_list(self):
		self.list_data = []
		self.list_data.append((None, 0, "> MIDI"))

		if zynthian_gui_config.midi_prog_change_zs3:
			self.list_data.append((self.toggle_prog_change_zs3, 0, "[x] Program Change ZS3"))
		else:
			self.list_data.append((self.toggle_prog_change_zs3, 0, "[  ] Program Change ZS3"))
			if zynthian_gui_config.midi_bank_change:
				self.list_data.append((self.toggle_bank_change, 0, "[x] MIDI Bank Change"))
			else:
				self.list_data.append((self.toggle_bank_change, 0, "[  ] MIDI Bank Change"))

		if zynthian_gui_config.preset_preload_noteon:
			self.list_data.append((self.toggle_preset_preload_noteon, 0, "[x] Preset Preload"))
		else:
			self.list_data.append((self.toggle_preset_preload_noteon, 0, "[  ] Preset Preload"))

		if zynthian_gui_config.midi_filter_output:
			self.list_data.append((self.toggle_midi_bridge_output, 0, "[x] Bridge MIDI-OUT"))
		else:
			self.list_data.append((self.toggle_midi_bridge_output, 0, "[  ] Bridge MIDI-OUT"))

		if zynthian_gui_config.transport_clock_source == 0:
			if zynthian_gui_config.midi_sys_enabled:
				self.list_data.append((self.toggle_midi_sys, 0, "[x] MIDI System Messages"))
			else:
				self.list_data.append((self.toggle_midi_sys, 0, "[  ] MIDI System Messages"))

		self.list_data.append((self.midi_profile, 0, "MIDI Profile"))

		self.list_data.append((None, 0, "> AUDIO"))

		if self.state_manager.allow_rbpi_headphones():
			if zynthian_gui_config.rbpi_headphones:
				self.list_data.append((self.stop_rbpi_headphones, 0, "[x] RBPi Headphones"))
			else:
				self.list_data.append((self.start_rbpi_headphones, 0, "[  ] RBPi Headphones"))

		if zynthian_gui_config.snapshot_mixer_settings:
			self.list_data.append((self.toggle_snapshot_mixer_settings, 0, "[x] Audio Levels on Snapshots"))
		else:
			self.list_data.append((self.toggle_snapshot_mixer_settings, 0, "[  ] Audio Levels on Snapshots"))

		if zynthian_gui_config.enable_dpm:
			self.list_data.append((self.toggle_dpm, 0, "[x] Mixer Peak Meters"))
		else:
			self.list_data.append((self.toggle_dpm, 0, "[  ] Mixer Peak Meters"))

		if zynconf.is_service_active("aubionotes"):
			self.list_data.append((self.state_manager.stop_aubionotes, 0, "[x] AubioNotes (Audio2MIDI)"))
		else:
			self.list_data.append((self.state_manager.start_aubionotes, 0, "[  ] AubioNotes (Audio2MIDI)"))

		self.list_data.append((None, 0, "> NETWORK"))

		self.list_data.append((self.network_info, 0, "Network Info"))

		if zynconf.is_wifi_active():
			if zynconf.is_service_active("hostapd"):
				self.list_data.append((self.state_manager.stop_wifi, 0, "[x] Wi-Fi Hotspot"))
			else:
				self.list_data.append((self.state_manager.stop_wifi, 0, "[x] Wi-Fi"))
		else:
			self.list_data.append((self.state_manager.start_wifi, 0, "[  ] Wi-Fi"))
			self.list_data.append((self.state_manager.start_wifi_hotspot, 0, "[  ] Wi-Fi Hotspot"))

		if zynconf.is_service_active("vncserver0"):
			self.list_data.append((self.state_manager.stop_vncserver, 0, "[x] VNC Server"))
		else:
			self.list_data.append((self.state_manager.start_vncserver, 0, "[  ] VNC Server"))

		if zynconf.is_service_active("jackrtpmidid"):
			self.list_data.append((self.state_manager.stop_rtpmidi, 0, "[x] RTP-MIDI"))
		else:
			self.list_data.append((self.state_manager.start_rtpmidi, 0, "[  ] RTP-MIDI"))

		if zynconf.is_service_active("qmidinet"):
			self.list_data.append((self.state_manager.stop_qmidinet, 0, "[x] QmidiNet (IP Multicast)"))
		else:
			self.list_data.append((self.state_manager.start_qmidinet, 0, "[  ] QmidiNet (IP Multicast)"))

		if zynconf.is_service_active("touchosc2midi"):
			self.list_data.append((self.state_manager.stop_touchosc2midi, 0, "[x] TouchOSC MIDI Bridge"))
		else:
			self.list_data.append((self.state_manager.start_touchosc2midi, 0, "[  ] TouchOSC MIDI Bridge"))

		self.list_data.append((None, 0, "> SETTINGS"))
		if self.zyngui.screens["brightness_config"].get_num_zctrls() > 0:
			self.list_data.append((self.zyngui.brightness_config, 0, "Brightness"))
		if "cv_config" in self.zyngui.screens:
			self.list_data.append((self.show_cv_config, 0, "CV Settings"))
		self.list_data.append((self.zyngui.calibrate_touchscreen, 0, "Calibrate Touchscreen"))
		self.list_data.append((self.zyngui.bluetooth_config, 0, "Bluetooth MIDI"))

		self.list_data.append((None, 0, "> TEST"))
		self.list_data.append((self.test_audio, 0, "Test Audio"))
		self.list_data.append((self.test_midi, 0, "Test MIDI"))

		self.list_data.append((None, 0, "> SYSTEM"))
		if self.zyngui.capture_log_fname:
			self.list_data.append((self.workflow_capture_stop, 0, "[x] Capture Workflow"))
		else:
			self.list_data.append((self.workflow_capture_start, 0, "[  ] Capture Workflow"))
		if self.is_update_available():
			self.list_data.append((self.update_software, 0, "Update Software"))
		else:
			self.list_data.append((self.check_for_updates, 0, "Check for software updates"))
		#self.list_data.append((self.update_system, 0, "Update Operating System"))
		#self.list_data.append((None, 0, "> POWER"))
		#self.list_data.append((self.restart_gui, 0, "Restart UI"))
		#self.list_data.append((self.exit_to_console, 0, "Exit to Console"))
		self.list_data.append((self.reboot, 0, "Reboot"))
		self.list_data.append((self.power_off, 0, "Power Off"))
		super().fill_list()

	def select_action(self, i, t='S'):
		if self.list_data[i][0]:
			self.last_action = self.list_data[i][0]
			self.last_action()

	def set_select_path(self):
		self.select_path.set("Admin")

	def execute_commands(self):
		self.state_manager.start_busy("admin_commands")
		error_counter = 0
		for cmd in self.commands:
			logging.info("Executing Command: %s" % cmd)
			self.zyngui.add_info("EXECUTING:\n", "EMPHASIS")
			self.zyngui.add_info("{}\n".format(cmd))
			try:
				self.proc = Popen(cmd, shell=True, stdout=PIPE, stderr=STDOUT, universal_newlines=True)
				self.zyngui.add_info("RESULT:\n", "EMPHASIS")
				for line in self.proc.stdout:
					if re.search("ERROR", line, re.IGNORECASE):
						error_counter += 1
						tag = "ERROR"
					elif re.search("Already", line, re.IGNORECASE):
						tag = "SUCCESS"
					else:
						tag = None
					logging.info(line.rstrip())
					self.zyngui.add_info(line, tag)
				self.zyngui.add_info("\n")
			except Exception as e:
				logging.error(e)
				self.zyngui.add_info("ERROR: %s\n" % e, "ERROR")

		if error_counter > 0:
			logging.info("COMPLETED WITH {} ERRORS!".format(error_counter))
			self.zyngui.add_info("COMPLETED WITH {} ERRORS!".format(error_counter), "WARNING")
		else:
			logging.info("COMPLETED OK!")
			self.zyngui.add_info("COMPLETED OK!", "SUCCESS")

		self.commands = None
		self.zyngui.add_info("\n\n")
		self.zyngui.hide_info_timer(5000)
		self.state_manager.end_busy("admin_commands")

	def start_command(self, cmds):
		if not self.commands:
			logging.info("Starting Command Sequence")
			self.commands = cmds
			self.thread = Thread(target=self.execute_commands, args=())
			self.thread.name = "command sequence"
			self.thread.daemon = True  # thread dies with the program
			self.thread.start()

	def killable_execute_commands(self):
		#self.state_manager.start_busy("admin commands")
		for cmd in self.commands:
			logging.info("Executing Command: %s" % cmd)
			self.zyngui.add_info("EXECUTING:\n", "EMPHASIS")
			self.zyngui.add_info("{}\n".format(cmd))
			try:
				proc = Popen(cmd.split(" "), stdout=PIPE, stderr=PIPE)
				self.child_pid = proc.pid
				self.zyngui.add_info("\nPID: %s" % self.child_pid)
				(output, error) = proc.communicate()
				self.child_pid = None
				if error:
					result = "ERROR: %s" % error
					logging.error(result)
					self.zyngui.add_info(result, "ERROR")
				if output:
					logging.info(output)
					self.zyngui.add_info(output)
			except Exception as e:
				result = "ERROR: %s" % e
				logging.error(result)
				self.zyngui.add_info(result, "ERROR")

		self.commands = None
		self.zyngui.hide_info_timer(5000)
		#self.state_manager.end_busy("admin commands")

	def killable_start_command(self, cmds):
		if not self.commands:
			logging.info("Starting Command Sequence")
			self.commands = cmds
			self.thread = Thread(target=self.killable_execute_commands, args=())
			self.thread.name = "killable command sequence"
			self.thread.daemon = True  # thread dies with the program
			self.thread.start()

	def kill_command(self):
		if self.child_pid:
			logging.info("Killing process %s" % self.child_pid)
			os.kill(self.child_pid, signal.SIGTERM)
			self.child_pid = None
			if self.last_action == self.test_midi:
				self.state_manager.all_sounds_off()

	# ------------------------------------------------------------------------------
	# CONFIG OPTIONS
	# ------------------------------------------------------------------------------

	def start_rbpi_headphones(self, save_config=True):
		logging.info("STARTING RBPI HEADPHONES")
		try:
			check_output("systemctl start headphones", shell=True)
			zynthian_gui_config.rbpi_headphones = 1
			# Update Config
			if save_config:
				zynconf.save_config({ 
					"ZYNTHIAN_RBPI_HEADPHONES": str(zynthian_gui_config.rbpi_headphones)
				})
			# Call autoconnect after a little time
			zynautoconnect.request_audio_connect()

		except Exception as e:
			logging.error(e)

		self.fill_list()

	def stop_rbpi_headphones(self, save_config=True):
		logging.info("STOPPING RBPI HEADPHONES")

		try:
			check_output("systemctl stop headphones", shell=True)
			zynthian_gui_config.rbpi_headphones = 0
			# Update Config
			if save_config:
				zynconf.save_config({ 
					"ZYNTHIAN_RBPI_HEADPHONES": str(int(zynthian_gui_config.rbpi_headphones))
				})

		except Exception as e:
			logging.error(e)

		self.fill_list()

	# Start/Stop RBPI Headphones depending on configuration
	def default_rbpi_headphones(self):
		if zynthian_gui_config.rbpi_headphones:
			self.start_rbpi_headphones(False)
		else:
			self.stop_rbpi_headphones(False)

	def toggle_dpm(self):
		zynthian_gui_config.enable_dpm = not zynthian_gui_config.enable_dpm
		self.fill_list()

	def toggle_snapshot_mixer_settings(self):
		if zynthian_gui_config.snapshot_mixer_settings:
			logging.info("Mixer Settings on Snapshots OFF")
			zynthian_gui_config.snapshot_mixer_settings = False
		else:
			logging.info("Mixer Settings on Snapshots ON")
			zynthian_gui_config.snapshot_mixer_settings = True

		# Update Config
		zynconf.save_config({ 
			"ZYNTHIAN_UI_SNAPSHOT_MIXER_SETTINGS": str(int(zynthian_gui_config.snapshot_mixer_settings))
		})
		self.fill_list()

	def toggle_midi_bridge_output(self):
		if zynthian_gui_config.midi_filter_output:
			logging.info("MIDI Bridge Output OFF")
			zynthian_gui_config.midi_filter_output = False
		else:
			logging.info("MIDI Bridge Output ON")
			zynthian_gui_config.midi_filter_output = True

		# Update MIDI profile
		zynconf.update_midi_profile({ 
			"ZYNTHIAN_MIDI_FILTER_OUTPUT": str(int(zynthian_gui_config.midi_filter_output))
		})
		zynautoconnect.request_midi_connect()
		self.fill_list()

	def toggle_midi_sys(self):
		if zynthian_gui_config.midi_sys_enabled:
			logging.info("MIDI System Messages OFF")
			zynthian_gui_config.midi_sys_enabled = False
		else:
			logging.info("MIDI System Messages ON")
			zynthian_gui_config.midi_sys_enabled = True

		# Update MIDI profile
		zynconf.update_midi_profile({ 
			"ZYNTHIAN_MIDI_SYS_ENABLED": str(int(zynthian_gui_config.midi_sys_enabled))
		})

		get_lib_zyncore().set_midi_filter_system_events(zynthian_gui_config.midi_sys_enabled)
		self.fill_list()

	def toggle_prog_change_zs3(self):
		if zynthian_gui_config.midi_prog_change_zs3:
			logging.info("ZS3 Program Change OFF")
			zynthian_gui_config.midi_prog_change_zs3 = False
		else:
			logging.info("ZS3 Program Change ON")
			zynthian_gui_config.midi_prog_change_zs3 = True

		# Save config
		zynconf.update_midi_profile({ 
			"ZYNTHIAN_MIDI_PROG_CHANGE_ZS3": str(int(zynthian_gui_config.midi_prog_change_zs3))
		})

		self.fill_list()

	def toggle_bank_change(self):
		if zynthian_gui_config.midi_bank_change:
			logging.info("MIDI Bank Change OFF")
			zynthian_gui_config.midi_bank_change = False
		else:
			logging.info("MIDI Bank Change ON")
			zynthian_gui_config.midi_bank_change = True

		# Save config
		zynconf.update_midi_profile({ 
			"ZYNTHIAN_MIDI_BANK_CHANGE": str(int(zynthian_gui_config.midi_bank_change))
		})

		self.fill_list()

	def toggle_preset_preload_noteon(self):
		if zynthian_gui_config.preset_preload_noteon:
			logging.info("Preset Preload OFF")
			zynthian_gui_config.preset_preload_noteon = False
		else:
			logging.info("Preset Preload ON")
			zynthian_gui_config.preset_preload_noteon = True

		# Save config
		zynconf.update_midi_profile({ 
			"ZYNTHIAN_MIDI_PRESET_PRELOAD_NOTEON": str(int(zynthian_gui_config.preset_preload_noteon))
		})
		self.fill_list()

	def show_cv_config(self):
		self.zyngui.show_screen("cv_config")

	def midi_profile(self):
		logging.info("MIDI Profile")
		self.zyngui.show_screen("midi_profile")

	# ------------------------------------------------------------------------------
	# NETWORK INFO
	# ------------------------------------------------------------------------------

	def network_info(self):
		self.zyngui.show_info("NETWORK INFO\n")

		res = zynconf.network_info()
		for k, v in res.items():
			self.zyngui.add_info(" {} => {}\n".format(k, v[0]), v[1])

		self.zyngui.hide_info_timer(5000)
		self.zyngui.state_manager.end_busy("gui_admin")

	# ------------------------------------------------------------------------------
	# TEST FUNCTIONS
	# ------------------------------------------------------------------------------

	def test_audio(self):
		logging.info("TESTING AUDIO")
		self.zyngui.show_info("TEST AUDIO")
		#self.killable_start_command(["mpg123 {}/audio/test.mp3".format(self.data_dir)])
		self.killable_start_command(["mplayer -nogui -noconsolecontrols -nolirc -nojoystick -really-quiet -ao jack {}/audio/test.mp3".format(self.data_dir)])
		zynautoconnect.request_audio_connect()

	def test_midi(self):
		logging.info("TESTING MIDI")
		self.zyngui.show_info("TEST MIDI")
		self.killable_start_command(["aplaymidi -p 14 {}/mid/test.mid".format(self.data_dir)])

		if self.zyngui.capture_log_fname:
			self.list_data.append((self.workflow_capture_stop, 0, "[x] Workflow Capture"))
		else:
			self.list_data.append((self.workflow_capture_start, 0, "[  ] Workflow Capture"))

	# ------------------------------------------------------------------------------
	# SYSTEM FUNCTIONS
	# ------------------------------------------------------------------------------

	def workflow_capture_start(self):
		self.zyngui.start_capture_log()
		self.zyngui.close_screen()

	def workflow_capture_stop(self):
		self.zyngui.stop_capture_log()
		self.fill_list()

	def update_software(self):
		logging.info("UPDATE SOFTWARE")
		self.zyngui.show_info("UPDATE SOFTWARE")
		self.start_command([self.sys_dir + "/scripts/update_zynthian.sh"])

	def is_update_available(self):
		repos = ["/zynthian/zyncoder", "/zynthian/zynthian-ui", "/zynthian/zynthian-sys", "/zynthian/zynthian-webconf", "/zynthian/zynthian-data"]
		update_available = False
		for path in repos:
			update_available |= (check_output("git -C %s status --porcelain -bs | grep behind | wc -l" % path, shell=True).decode()[:1] == '1')
		return update_available

	def check_for_updates(self):
		self.zyngui.show_info("CHECK FOR UPDATES")
		self.start_command(["git -C /zynthian/zyncoder remote update; git -C /zynthian/zynthian-ui remote update; git -C /zynthian/zynthian-sys remote update; git -C /zynthian/zynthian-webconf remote update; git -C /zynthian/zynthian-data remote update"])

	def update_system(self):
		logging.info("UPDATE SYSTEM")
		self.zyngui.show_info("UPDATE SYSTEM")
		self.start_command([self.sys_dir + "/scripts/update_system.sh"])

	def restart_gui(self):
		logging.info("RESTART ZYNTHIAN-UI")
		self.zyngui.show_splash("Restarting UI")
		self.last_state_action()
		self.zyngui.exit(102)

	def exit_to_console(self):
		logging.info("EXIT TO CONSOLE")
		self.zyngui.show_splash("Exiting")
		self.last_state_action()
		self.zyngui.exit(101)

	def reboot(self):
		self.zyngui.show_confirm("Do you really want to reboot?", self.reboot_confirmed)

	def reboot_confirmed(self, params=None):
		logging.info("REBOOT")
		self.zyngui.show_splash("Rebooting")
		self.last_state_action()
		self.zyngui.exit(100)

	def power_off(self):
		self.zyngui.show_confirm("Do you really want to power off?", self.power_off_confirmed)

	def power_off_confirmed(self, params=None):
		logging.info("POWER OFF")
		self.zyngui.show_splash("Powering Off")
		self.last_state_action()
		self.zyngui.exit(0)

	def last_state_action(self):
		if zynthian_gui_config.restore_last_state:
			self.state_manager.save_last_state_snapshot()
		else:
			self.state_manager.delete_last_state_snapshot()

# ------------------------------------------------------------------------------
