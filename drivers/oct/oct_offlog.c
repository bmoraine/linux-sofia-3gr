/*
 * Copyright (C) 2014 Intel Mobile Communications GmbH
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/syscalls.h>

/* Global variables */
loff_t pos = 0;

/* offline bplog definition */
#define OFFLOG_PATH_LEN 255

#define OFFLOG_CONFIG_FILE "/system/etc/oct_offlog.conf"
#define OFFLOG_CONFIG_FILE_SIZE 500

#define OFFLOG_CONFIG_TAG_ONOFF "on_off"
#define OFFLOG_CONFIG_TAG_PATH  "offlog_path"
#define OFFLOG_CONFIG_TAG_SIZE  "offlog_size"
#define OFFLOG_CONFIG_TAG_NUM   "offlog_number"
#define OFFLOG_CONFIG_TAG_RUN   "offlog_run"


#define OFFLOG_CONFIG_MAX_VALUE_LENGTH 50

/*#define DEFAULT_OCT_OFFLOG_SIZE 500000000*/
#define DEFAULT_OCT_OFFLOG_SIZE 10000000
#define DEFAULT_OCT_OFFLOG_NUMBER 5
#define DEFAULT_OFFLOG_PATH "/data/logs/"
#define DEFAULT_OFFLOG_NAME "bplog"

static char oct_offlog_path[OFFLOG_PATH_LEN+1] = {0}; /* /data/logs/bplog */
static unsigned int oct_offlog_onoff = 1;
static unsigned int oct_offlog_size = DEFAULT_OCT_OFFLOG_SIZE;
static unsigned int oct_offlog_number = DEFAULT_OCT_OFFLOG_NUMBER;
static unsigned int oct_offlog_run = 0;

static void oct_detect_off_flag(char *gadget_name)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	int retry = 12;
	OCT_DBG("oct_detect_off_flag[%s]", gadget_name);

	while (!kthread_should_stop() && retry >= 0) {
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		fp = filp_open(gadget_name, O_RDONLY, 0);
		if (NULL == fp) {
			OCT_LOG("NULL Gadget File Handle");
			msleep(2000);
			continue;
		} else if (IS_ERR(fp)) {
			OCT_LOG("Gadget fp=%x open failed!", (unsigned int)fp);
			retry--;
			msleep(2000);
			continue;
		} else {
			OCT_LOG("Gadget %s open succeeded. fp=%x",
					gadget_name, (unsigned int)fp);
			oct_out_path = OCT_PATH_FILE;
			filp_close(fp, NULL);
			break;
		}
		set_fs(old_fs);
		set_current_state(TASK_INTERRUPTIBLE);
	}
	return;
}

static int str2int(char *str)
{
	int value = 0;
	char *s = str;
	if (NULL == str || strlen(str) == 0)
		return 0;
	do {
		if (*s > '9' || *s < '0')
			continue;
		value *= 10;
		value += *s - '0';
	} while (*(++s) != 0);
	return value;
}

bool get_value_by_tag(char *config, char *tag, char *value, int size)
{
	int i = 0;
	char *tv = strstr(config, tag);
	if (value == NULL || size == 0)
		return false;
	if (tv != NULL && strlen(tv) != 0) {
		bool b_equal_symbol = false;
		while (*(++tv) != '\n' && *tv != '\0' && i < size) {
			if (!((*tv >= 'a' && *tv <= 'z') ||
				(*tv >= 'A' && *tv <= 'Z') ||
				(*tv >= '0' && *tv <= '9') ||
				*tv == '_' || *tv == '.' ||
				*tv == '=' || *tv == '/' ||
				*tv == '\\'))
				continue;

		if (b_equal_symbol)
			value[i++] = *tv;
		if (*tv == '=')
			b_equal_symbol = true;
		}
		value[i++] = '\0';
	}
	return true;
}

static bool oct_offlog_get_config(char *cfg_path)
{
	char cfg_context[OFFLOG_CONFIG_FILE_SIZE+1] = {0};
	char value_onoff[OFFLOG_CONFIG_MAX_VALUE_LENGTH+1] = {0};
	char value_path[OFFLOG_CONFIG_MAX_VALUE_LENGTH+1] = {0};
	char value_size[OFFLOG_CONFIG_MAX_VALUE_LENGTH+1] = {0};
	char value_num[OFFLOG_CONFIG_MAX_VALUE_LENGTH+1] = {0};
	char value_run[OFFLOG_CONFIG_MAX_VALUE_LENGTH+1] = {0};
	struct file *fp_cfg;
	loff_t pos_cfg = 0;
	/* loff_t pos_dir = 0; */
	mm_segment_t old_fs;
	int file_len = 0;
	OCT_DBG("oct_get_offlog_config");

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	fp_cfg = filp_open(cfg_path, O_RDONLY, 0);
	if (!IS_ERR_OR_NULL(fp_cfg)) {
		file_len = fp_cfg->f_op->llseek(fp_cfg, 0, SEEK_END);
		if (file_len > 0)
			fp_cfg->f_op->read(fp_cfg, cfg_context,
					file_len, &pos_cfg);
		if (strlen(cfg_context) != 0) {
			get_value_by_tag(cfg_context, OFFLOG_CONFIG_TAG_ONOFF,
				value_onoff, OFFLOG_CONFIG_MAX_VALUE_LENGTH);
			get_value_by_tag(cfg_context, OFFLOG_CONFIG_TAG_PATH,
				value_path, OFFLOG_CONFIG_MAX_VALUE_LENGTH);
			get_value_by_tag(cfg_context, OFFLOG_CONFIG_TAG_SIZE,
				value_size, OFFLOG_CONFIG_MAX_VALUE_LENGTH);
			get_value_by_tag(cfg_context, OFFLOG_CONFIG_TAG_NUM,
				value_num, OFFLOG_CONFIG_MAX_VALUE_LENGTH);
			get_value_by_tag(cfg_context, OFFLOG_CONFIG_TAG_RUN,
				value_run, OFFLOG_CONFIG_MAX_VALUE_LENGTH);
		}
		filp_close(fp_cfg, NULL);
		fp_cfg = NULL;
	} else
		OCT_LOG("oct_get_offlog_config config not exist");
	set_fs(old_fs);

	/* config onoff */
	oct_offlog_onoff = (value_onoff == NULL || strlen(value_onoff) == 0) ?
		0 : str2int(value_onoff);
	/* config run */
	oct_offlog_run = (value_run == NULL || strlen(value_run) == 0) ?
		0 : str2int(value_run);
	/* config path */
	memset(oct_offlog_path, 0, OFFLOG_PATH_LEN + 1);
	if (value_path == NULL || strlen(value_path) == 0)
		snprintf(oct_offlog_path, OFFLOG_PATH_LEN, "%s%s",
				DEFAULT_OFFLOG_PATH, DEFAULT_OFFLOG_NAME);
	else
		snprintf(oct_offlog_path, OFFLOG_PATH_LEN, "%s%s",
				value_path, DEFAULT_OFFLOG_NAME);
	/* config size */
	oct_offlog_size = str2int(value_size);
	if (oct_offlog_size == 0)
		oct_offlog_size = DEFAULT_OCT_OFFLOG_SIZE;
	/* config file numbers */
	oct_offlog_number = str2int(value_num);
	if (oct_offlog_number == 0)
		oct_offlog_number = DEFAULT_OCT_OFFLOG_NUMBER;
	OCT_LOG("oct_offlog_get_conf, onoff: %d, path: %s, size:%d, number:%d run:%d",
		oct_offlog_onoff, oct_offlog_path,
		oct_offlog_size, oct_offlog_number,
		oct_offlog_run);
	return true;
}

void oct_offlog_check_mk_dirs(void)
{
	mm_segment_t old_fs;
	int offlog_path_length = strlen(oct_offlog_path);
	char c = '/';
	char *nextdir = NULL;

	nextdir = strchr(oct_offlog_path + 1, c);
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	while (NULL != nextdir && strlen(nextdir) != 0) {
		char curdir[OFFLOG_PATH_LEN+1] = {0};
		strncpy(curdir, oct_offlog_path,
				offlog_path_length - strlen(nextdir) + 1);
		/* TODO: test if curdir exist. if not exist, create it. */
		sys_mkdir(curdir, 0777);
		nextdir = strchr(nextdir+1, c);
	}
	set_fs(old_fs);
}

static bool oct_offlog_if_rotate(void)
{
	/* if bplog exist */
	if (NULL == oct_offlog_path) {
		OCT_LOG("oct_offlog_if_rotate offlog path null");
		return false;
	}
	/*  TODO: check if bplog exist to deside if need rotate */
	/* err = sys_access(oct_offlog_path, R_OK);
	if (!err) {
		OCT_LOG("oct_offlog_if_rotate offlog not exist");
		return false;
	} */
	OCT_DBG("oct_offlog_if_rotate return true");
	return true;
}

static int oct_offlog_rotate_files(void)
{
	mm_segment_t old_fs;
	int err, i;
	char path_del[OFFLOG_PATH_LEN+1] = {0};
	OCT_DBG("oct_offlog_rotate_files");
	/* remove bplog.4.istp */
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	snprintf(path_del, OFFLOG_PATH_LEN, "%s.%d.istp", oct_offlog_path,
			oct_offlog_number-1);
	err = sys_unlink(path_del);
	/* rename other files */
	for (i = oct_offlog_number-2; i >= 0; i--) {
		char oldpath[OFFLOG_PATH_LEN+1] = {0};
		char newpath[OFFLOG_PATH_LEN+1] = {0};
		if (i == 0)
			snprintf(oldpath, OFFLOG_PATH_LEN, "%s",
					oct_offlog_path);
		else
			snprintf(oldpath, OFFLOG_PATH_LEN, "%s.%d.istp",
					oct_offlog_path, i);
		snprintf(newpath, OFFLOG_PATH_LEN, "%s.%d.istp",
				oct_offlog_path, i+1);
		sys_rename(oldpath, newpath);
	}
	set_fs(old_fs);
	return 1;
}

static void oct_offlog_chat_cmd(const char *devname, const char *atcmd)
{
	mm_segment_t oldfs;
	struct file *filp;
	unsigned int writed = 0;
	#define AT_CMD_LEN 64
	char readbuf[AT_CMD_LEN] = {0};
	unsigned int wait_ms = 200, max_wait_times = 5, wait_times = 0;

	oldfs = get_fs();
	set_fs(KERNEL_DS);

	filp = filp_open(devname, O_RDWR, 0);

	if (!filp || IS_ERR(filp))
		OCT_LOG("can not open [%s],filp=%ld\n", devname, PTR_ERR(filp));
	else {
		writed = filp->f_op->write(filp, atcmd, strlen(atcmd),
				&filp->f_pos);
		OCT_DBG("write[%s][%d]: [%s]", devname, writed, atcmd);
		do {
			msleep(wait_ms);
			filp->f_op->read(filp, readbuf, AT_CMD_LEN - 1,
					&filp->f_pos);
		OCT_DBG("read[%s]: [%s]", devname, readbuf);
		} while ((strlen(readbuf) == 0) &&
				(wait_times++ < max_wait_times));
		filp_close(filp, NULL);
	}
	set_fs(oldfs);
}

static int oct_offlog_send_init_at(struct device_node *np)
{
	int i, ret = 0, len;
	const char *pipe;
	if (of_property_read_string(np, "oct,offlog_pipe", &pipe))
		return 0;
	pr_info("%s: pipe %s found!\n", __func__, pipe);
	len = of_property_count_strings(np, "oct,offlog_cmds");
	if (!len)
		return 0;
	pr_info("%s: %d at commands found!\n", __func__, len);
	for (i = 0; i < len; i++) {
		const char *at;
		of_property_read_string_index(np, "oct,offlog_cmds", i, &at);
		oct_offlog_chat_cmd(pipe, at);
	}
	return ret;
}

void oct_write_data_to_file(void *ptr, int num_bytes)
{
	mm_segment_t old_fs;
	int written_bytes = 0;
	int verify_fp = 2;
	static int total_written_bytes;

	if (!oct_offlog_onoff && oct_offlog_run) {
		written_bytes = num_bytes;
		goto update_ptr;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	if (total_written_bytes >= oct_offlog_size) {
		if (!IS_ERR_OR_NULL(fp)) {
			filp_close(fp, NULL);
			fp = NULL;
			pos = 0;
		}
		oct_offlog_rotate_files();
		total_written_bytes = 0;
	}
	while (verify_fp > 0) {
		if (IS_ERR_OR_NULL(fp)) {
			OCT_LOG("Gadget fp=%p is not valid=%ld!",
					fp, PTR_ERR(fp));
			/* re-open the tty gadget */
			fp = filp_open(oct_offlog_path, O_RDWR | O_CREAT |
					O_TRUNC, 0666);
		}
		if (IS_ERR_OR_NULL(fp)) {
			OCT_LOG("Gadget fp=%p re-open failed with error=%ld!",
						fp, PTR_ERR(fp));
			/* skip bytes in buffer and leave loop */
			written_bytes = num_bytes;
			verify_fp = -1;
		} else {
			/* try a first time write */
			written_bytes =
				fp->f_op->write(fp,
						(char *)ptr, num_bytes, &pos);
			if (written_bytes <= 0) {
				OCT_LOG(
					"Error: file write Error with valid fp=%p, %d",
					fp, written_bytes);
				/* come back and force the loop to re-open fp */
				fp = NULL;
				/* avoid infinity loop doing only
				 * one time the loop */
				verify_fp--;
				/* skip bytes in buffer and leave loop */
				written_bytes = num_bytes;
			} else {
				/* write sucessfuly to USB and leave the loop */
				verify_fp = -1;
				total_written_bytes += written_bytes;
			}
		}
	}
	set_fs(old_fs);

update_ptr:
	/* increment the read ptr anyhow to re-start timeout */
	oct_read_ptr += written_bytes;
	/* check if result is consistent */
	if (oct_read_ptr > oct_ext_ring_buff_len) {
		/* too many bytes */
		OCT_LOG("Error: ReadPtr exceeds the buffer length");
		OCT_LOG("written bytes 0x%x, oct_read_ptr 0x%x",
				written_bytes, oct_read_ptr);
		oct_read_ptr = 0;
	} else if (oct_read_ptr == oct_ext_ring_buff_len) {
		oct_read_ptr = 0; /*wrap around*/
		OCT_DBG("ReadPtr wrap around:");
		OCT_DBG(" written bytes 0x%x, oct_read_ptr 0x%x",
				written_bytes, oct_read_ptr);
	}
	SET_OCT_OCT_SW_RPTR(oct_trform, oct_read_ptr);
	data_available = 0;
	/* reset & enable CYCLE interrupt */
	SET_OCT_OCT_MASTER_TRIG_CYCLE_DMA_TRIG_CYCLE(oct_trform,
							   current_timeout);
	SET_OCT_OCT_CNF_ENABLE_TRIG_CYCLE_INT(oct_trform,
				    OCT_CNF_ENABLE_TRIG_CYCLE_INT_ENABLED);
}

static const struct file_operations oct_proc_fops = {
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

void oct_offlog_config(struct device_node *np)
{
	oct_detect_off_flag(OFFLOG_CONFIG_FILE);
	OCT_DBG("oct_detect_off_flag oct_out_path: %d", oct_out_path);

	if (oct_out_path == OCT_PATH_FILE) {
		oct_offlog_get_config(OFFLOG_CONFIG_FILE);
		if (!oct_offlog_onoff && !oct_offlog_run)
			oct_out_path = DEFAULT_OCT_PATH;
	}
	OCT_DBG("oct_detect_off_flag oct_out_path: %d", oct_out_path);
	OCT_DBG("oct_out_path %d", oct_out_path);

	if (oct_out_path == OCT_PATH_FILE) {
		if (oct_offlog_onoff) {
			oct_offlog_check_mk_dirs();
			if (oct_offlog_if_rotate())
				oct_offlog_rotate_files();
			oct_offlog_send_init_at(np);
			fp = oct_open_gadget(oct_offlog_path);
		}
	}
}

