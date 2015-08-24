ifeq ($(CONFIG_VIDEO_XGOLD_FLASH),y)
export EXTRA_CFLAGS += -DCONFIG_CAMERA_FLASH_SUPPORT
endif

obj-$(CONFIG_VIDEO_GC0310) += sofia-3g_camera_module.o gc_common.o gc_0310.o
obj-$(CONFIG_VIDEO_GC2155) += sofia-3g_camera_module.o gc_common.o gc_2155.o
obj-$(CONFIG_VIDEO_GC2355) += gc_camera_module.o sofia-3g_camera_module.o gc2355_v4l2-i2c-subdev.o
obj-$(CONFIG_VIDEO_XGOLD_GC2355) += sofia-3g_camera_module.o gc2355.o
obj-$(CONFIG_VIDEO_GC0310_RAW) += sofia-3g_camera_module.o gc_camera_module.o gc0310_v4l2-i2c-subdev.o

obj-$(CONFIG_VIDEO_OV5670) += ov_camera_module.o sofia-3g_camera_module.o ov5670_v4l2-i2c-subdev.o
obj-$(CONFIG_VIDEO_OV8825) += ov_camera_module.o sofia-3g_camera_module.o ov8825_v4l2-i2c-subdev.o
obj-$(CONFIG_VIDEO_OV9740) += ov_camera_module.o sofia-3g_camera_module.o ov9740_v4l2-i2c-subdev.o
obj-$(CONFIG_VIDEO_OV9760) += ov_camera_module.o sofia-3g_camera_module.o ov9760_v4l2-i2c-subdev.o
obj-$(CONFIG_VIDEO_OV2680) += ov_camera_module.o sofia-3g_camera_module.o ov2680_v4l2-i2c-subdev.o
obj-$(CONFIG_VIDEO_OV2685) += ov_camera_module.o sofia-3g_camera_module.o ov2685_v4l2-i2c-subdev.o
obj-$(CONFIG_VIDEO_OV13850) += wv511_v4l2-i2c-subdev.o ad5823_v4l2-i2c-subdev.o ov_camera_module.o sofia-3g_camera_module.o ov13850_v4l2-i2c-subdev.o
obj-$(CONFIG_VIDEO_OV5648) += wv511_v4l2-i2c-subdev.o ov_camera_module.o sofia-3g_camera_module.o ov5648_v4l2-i2c-subdev.o
obj-$(CONFIG_VIDEO_OV8858) += vm149c_v4l2-i2c-subdev.o wv511_v4l2-i2c-subdev.o ov_camera_module.o sofia-3g_camera_module.o ov8858_v4l2-i2c-subdev.o

obj-$(CONFIG_VIDEO_HM2051) += sofia-3g_camera_module.o hm2051.o
