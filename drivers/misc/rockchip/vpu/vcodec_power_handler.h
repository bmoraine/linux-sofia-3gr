#ifndef _VCODEC_POWER_HANDLER_H
#define _VCODEC_POWER_HANDLER_H

int vpu_power_req_vbpipe_init(void);
int vpu_suspend_req(void);
int vpu_resume_req(void);
int vpu_power_req_vbpipe_close(void);

#endif
