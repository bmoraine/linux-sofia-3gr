# Copyright (C) 2011-2014 Intel Mobile Communications GmbH.
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License Version 2
#  as published by the Free Software Foundation.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
#
#  You should have received a copy of the GNU General Public License Version 2
#  along with this program. If not, see <http://www.gnu.org/licenses/>.

snd-soc-xgold-pcm-objs := xgold_pcm.o xgold_voice.o
snd-soc-xgold-speech-objs := xgold_speech_probe.o
snd-soc-xgold-machine-objs := xgold_machine.o xgold_jack.o
snd-soc-agold-bt-sco-streaming-objs := agold_bt_sco_streaming.o
snd-soc-xgold-dsp-objs :=  dsp_audio_platform.o

# dsp driver probe should be called before to other device probes
obj-$(CONFIG_SND_XGOLD_DSP) += snd-soc-xgold-dsp.o
obj-$(CONFIG_SND_XGOLD_DSP) += xgold_dsp/

obj-$(CONFIG_SND_SOC_XGOLD_MACHINE) += snd-soc-xgold-machine.o
obj-$(CONFIG_SND_SOC_XGOLD_PCM) += snd-soc-xgold-pcm.o
obj-$(CONFIG_SND_SOC_XGOLD_SPEECH_PROBE) += snd-soc-xgold-speech.o
obj-$(CONFIG_SND_SOC_AGOLD_BT_SCO_STREAMING) += snd-soc-agold-bt-sco-streaming.o

ccflags-$(CONFIG_SND_SOC_XGOLD_MACHINE) += -I$(srctree)/sound/soc/xgold/include
ccflags-$(CONFIG_SND_XGOLD_DSP) += -I$(srctree)/sound/soc/xgold/xgold_dsp/include
