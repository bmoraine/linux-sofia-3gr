/*
 * Copyright (c) 2015, Intel Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#ifndef APP_MODE
#include <linux/spinlock.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <asm/uaccess.h>
#include <linux/smp.h>
#include <linux/slab.h>
#include "include/vmm_hsym.h"
#include "include/vmm_hsym_common.h"
#include "include/vidt_ioctl.h"
//#include "include/types.h"
#include "include/sl_types.h"
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/notifier.h>
#endif

#define ACTIVATE_ACR3

__asm__ ( \
		".intel_syntax noprefix\n"  \
		"patch_str_secs_ptr = 0xAABBCCDD\n" \
		"patch_str_core_id =  0xBBCCDDAA\n" \
		"patch_str_core_state_ptr = 0xCCDDAABB\n" \
		"patch_str_secs_scv = 0xDDCCAABB\n" \
		"patch_str_region_addr = 0xAACCBBDD\n" \
		"patch_str_idt_vector = 0xAADDBBCC\n"   \
		"patch_str_enter_page = 0xBBDDAACC\n" \
		"patch_str_exit_page = 0xCCAADDBB\n" \
		"patch_str_secs_scv_un = 0xDDBBAACC\n" \
		".att_syntax\n" \
	);

// ***********************************************
//!!!!!!!!!!!!!!!!!! NOTE WELL !!!!!!!!!!!!!!!!!!!
//if this code is changed, the entrypoint
//offset added to the my_handler base in read_idt
//must be updated to ensure the correct entrypoints
//are registered in the vIDT
//Also NOTE - mov for val is used to ensure opcode
//does not change with operand size, so that size
//of this code block does not change
// ************************************************
void vidt_stub_patch_val0(void);
void begin_vidt_stub0(void);
void vidt_stub_patch_callee0(void);
#define VIDT_STUB(errcode,val) \
	__asm__ ( \
			".intel_syntax noprefix\n" \
			".globl begin_vidt_stub0\n" \
			".globl vidt_stub_patch_val0\n" \
			".globl vidt_stub_patch_callee0\n" \
			"begin_vidt_stub"val":\n"  \
			"push edi;\n" \
			"push esi;\n" \
			"push ebx;\n" \
			"push edx;\n" \
			"push ecx;\n" \
			"push eax;\n" \
			\
			"vidt_stub_patch_val"val":\n"   \
			"mov eax, patch_str_idt_vector;\n" \
			"push eax;\n" \
			"push "errcode";\n" \
			"vidt_stub_patch_callee"val":\n"   \
			"mov eax, patch_str_region_addr;\n" \
			"call eax;\n" \
			\
"cmp eax, 0;\n" \
"je vidt_stub_use_iret"val";\n" \
"pop eax;\n" \
"pop ecx;\n" \
"pop edx;\n" \
"pop ebx;\n" \
"pop esi;\n" \
"xchg edi, [esp];\n" \
"ret;\n" \
\
"vidt_stub_use_iret"val":\n" \
"pop eax;\n" \
"pop ecx;\n" \
"pop edx;\n" \
"pop ebx;\n" \
"pop esi;\n" \
"pop edi;\n" \
"iret;\n" \
".att_syntax\n" \
);

void my_handler(void);

__asm__ (
		".intel_syntax noprefix\n"
		".globl my_handler\n"

		"my_handler:\n"
		//#DE
		"vidt_entry_0:\n"
	);
VIDT_STUB("0","0");
__asm__ (
		//#DB
		"vidt_entry_1:\n"
	);
VIDT_STUB("0","1");
__asm__ (
		//#NMI
		"vidt_entry_2:\n"
	);
VIDT_STUB("0","2");
__asm__ (
		//#BP
		"vidt_entry_3:\n"
	);
VIDT_STUB("0","3");
__asm__ (
		//#OF
		"vidt_entry_4:\n"
	);
VIDT_STUB("0","4");
__asm__ (
		//#BR
		"vidt_entry_5:\n"
	);
VIDT_STUB("0","5");
__asm__ (
		//#UD
		"vidt_entry_6:\n"
	);
VIDT_STUB("0","6");
__asm__ (
		//#NM
		"vidt_entry_7:\n"
	);
VIDT_STUB("0","7");
__asm__ (
		//#DF
		"vidt_entry_8:\n"
	);
VIDT_STUB("1","8"); //errcode=1
__asm__ (
		//CSO Abort
		"vidt_entry_9:\n"
	);
VIDT_STUB("0","9");
__asm__ (
		//#TS
		"vidt_entry_10:\n"
	);
VIDT_STUB("1","10"); //errcode=1
__asm__ (
		//#NP
		"vidt_entry_11:\n"
	);
VIDT_STUB("1","11"); //errcode=1
__asm__ (
		//#SS
		"vidt_entry_12:\n"
	);
VIDT_STUB("1","12"); //errcode=1
__asm__ (
		//#GP
		"vidt_entry_13:\n"
	);
VIDT_STUB("1","13"); //errcode=1
__asm__ (
		//#PF
		"vidt_entry_14:\n"
	);
VIDT_STUB("1","14"); //errcode=1
__asm__ (
		//RESV
		"vidt_entry_15:\n"
	);
VIDT_STUB("0","15");
__asm__ (
		//#MF
		"vidt_entry_16:\n"
	);
VIDT_STUB("0","16");
__asm__ (
		//#AC
		"vidt_entry_17:\n"
	);
VIDT_STUB("1","17"); //errcode=1
__asm__ (
		//#MC
		"vidt_entry_18:\n"
	);
VIDT_STUB("0","18");
__asm__ (
		//#XM
		"vidt_entry_19:\n"
	);
VIDT_STUB("0","19");
__asm__ (
		//#VE
		"vidt_entry_20:\n"
	);
VIDT_STUB("0","20");
///////////////////////////////// arch handlers end
//TBD NEED TO FILL IN OTHER ISRS uptil index 255
//THOSE ARE COPIED into memory in the code
//that sets up the per-core vIDT code pages
__asm__ (
		".att_syntax\n"
	);
void my_handler_arch_end(void);
__asm__ (
		".globl my_handler_arch_end\n"
		"my_handler_arch_end:"
	);
VIDT_STUB("0","99"); //dummy ISR for size calculation
void my_handler_non_arch_end(void);
__asm__ (
		".globl my_handler_non_arch_end\n"
		"my_handler_non_arch_end:"
		"nop;\n"
	);

//AEX - ASYNCHRONOUS EXIT VIEW FLOW
//NOTE - expected to be called only from my_handler code
//Expects to get a flag param on stack to skip error code
//based on IDT vector - some entries below 20 have error code
void test_code_cpuindex(void);
void test_code_ptr_core_state_patch(void);
void test_code_secs_patch1(void);
void test_code_secs_patch2(void);
void test_code_cmp_patch(void);
void test_code_exit_page_patch(void);
void test_code(void);
__asm__ (
		".intel_syntax noprefix\n"
		".globl test_code_cpuindex\n"
		".globl test_code_ptr_core_state_patch\n"
		".globl test_code_secs_patch1\n"
		".globl test_code_secs_patch2\n"
		".globl test_code_cmp_patch\n"
		".globl test_code_exit_page_patch\n"
		".globl test_code\n"

		"params_size           =8\n"
		//stack params offsets
		"flag                  =4\n"
		"vector                =8\n"

		"gpr_frame_size        =24\n"
		//offset to gprs on stack
		"start_gprs            =params_size + 4\n"
		//offsets for gprs with base at start_gprs
		"gprs.eax               =0\n"
		"gprs.ecx               =4\n"
		"gprs.edx               =8\n"
		"gprs.ebx               =12\n"
		"gprs.esi               =16\n"
		"gprs.edi               =20\n"
		"ur0sp                  =96\n"
		//intr_frame offsets
		"if_skip_ec            =start_gprs + gpr_frame_size + 4\n" //skips errcode
		"if_noskip_ec          =start_gprs + gpr_frame_size\n"   //does not skip errcode
		"if.errcode            =0\n" //used with if_noskip_ec offset
		//these offsets are with reference base if_skip_ec and if_noskip_ec:
		"if.eip                =0\n"
		"if.cs                 =4\n"
		"if.eflags             =8\n"
		"if.esp                =12\n"
		"if.ss                 =16\n"

		//secs offsets from arch72.h
		"secs.size             =0\n"
		"secs.base             =8\n"
		"secs.ssa_frame_size   =16\n"
		"secs.attrib           =48\n"
		"secs.eid              =260\n"
		"secs.ept_enabled      =3559\n"
		"secs.acr3             =3560\n"
		"secs.scv              =3568\n"
		"secs.os_cr3           =3576\n"
		"secs.pcd              =3584\n" //per core data
		//secs offsets for per core data
		"secs.pcd.tcs          =0\n"
		"secs.pcd.idtr         =8\n"
		"secs.pcd.gdtr         =16\n"
		"secs.pcd.ldtr         =24\n"
		"secs.pcd.tr           =32\n"
		"secs.pcd.r0sp         =40\n"

		"shift_size_secs_pcd   =6\n" //pcd size is 64 currently
		"shift_size_4k         =12\n" //ssa frame size is 4k

		//tcs offsets from arch72.h
		"tcs.state             =0\n"
		"tcs.flags             =8\n"
		"tcs.ossa              =16\n"
		"tcs.cssa              =24\n"
		"tcs.nssa              =28\n"
		"tcs.oentry            =32\n"
		"tcs.aep               =40\n"
		"tcs.ofs_base          =48\n"
		"tcs.ogs_base          =56\n"
		"tcs.ofs_limit         =64\n"
		"tcs.ogs_limit         =68\n"
		"tcs.save_fs_selector  =72\n"
		"tcs.save_fs_desc_low  =74\n"
		"tcs.save_fs_desc_high =78\n"
		"tcs.save_gs_selector  =82\n"
		"tcs.save_gs_desc_low  =84\n"
		"tcs.save_gs_desc_high =88\n"
		"tcs.ssa               =92\n"
		"tcs.eflags            =96\n"
		"tcs.os_cr3            =100\n"
		"tcs.ur0sp             =108\n"
		"tcs.tr0sp             =116\n"

		"tcs_state_expect_active =1\n"
		"tcs_state_set_inactive  =0\n"

		//note ssa gpr area at end of page
		"ssa_gpr_size          =168\n"
		//ssa offsets from arch72.h
		"ssa.ax                =0\n"
		"ssa.cx                =8\n"
		"ssa.dx                =16\n"
		"ssa.bx                =24\n"
		"ssa.sp                =32\n"
		"ssa.bp                =40\n"
		"ssa.si                =48\n"
		"ssa.di                =56\n"
		"ssa.flags             =128\n"
		"ssa.ip                =136\n"
		"ssa.sp_u              =144\n"
		"ssa.bp_u              =152\n"
		"cr0.ts                 =3\n"
		"seg_granularity       =23\n"

		"vmfunc_view_sw_ctrl   =0\n"
		"vmfunc_return_success =0\n"

		"untrusted_view_id     =0\n"

		"service_type          =24\n"
		"nr                    =999\n"
		"vmcall_assert         =42\n"
		"vmcall_ta_exception   =0x9f\n"

		"size_core_state       =22\n" // look at PER_CORE_STATE data structure
		"redirect_table        =12\n"
		"sizeof_redirect_entry =8\n"

		"eresume_leaf          =3\n"
		"eenter_opsize         =2\n"

		"eenter_vector         =29\n"
		"eresume_vector        =30\n"
		"eexit_vector          =31\n"
		"kenter_vector         =28\n"
		"kexit_vector          =27\n"

		"ss_rpl_mask           =0x0003\n"
		"ss_rpl_ring3          =0x0003\n"
		"ss_rpl_ring0          =0x0000\n"
		"ss_ti_mask            =0x0004\n"
		"ss_ti_gdt             =0x0000\n"
		"ss_ti_ldt             =0x0004\n"
		"enclave_crashed       =0x1006\n" //error code
		"ecall_not_allowed     =0x1007\n" //error code

		"exception_pf          =14\n"
		"exception_last        =31\n"
		"exception_nmi         =2\n"
		"exception_mc          =18\n"
		"view_0_chk_fail       =0x2\n"
		"secs_scv_chk_fail     =0x3\n"
		"trusted_view_chk_fail =0x4\n"
		"trusted_view_init_fail =0x5\n"
		"tcs_pg_align_chk_fail =0x6\n"
		"ssa_frame_avl_chk1_fail =0x7\n"
		"ssa_frame_avl_chk2_fail =0x8\n"
		"aep_chk_fail            =0x9\n"
		"target_chk_fail         =0xa\n"
		"ss_db_chk_fail          =0xb\n"
		"ds_expand_chk_fail      =0xc\n"
		"ossa_page_align_chk_fail=0xd\n"
		"ofsbase_page_align_chk_fail=0xe\n"
		"ogsbase_page_align_chk_fail=0xf\n"
		"no_fs_wrap_around_chk_fail=0x10\n"
		"fs_within_ds_chk_fail     =0x11\n"
		"no_gs_wrap_around_chk_fail=0x12\n"
		"gs_within_ds_chk_fail     =0x13\n"
		"tcs_reserved_chk_fail     =0x14\n"
		"tcs_lock_acquire_fail     =0x15\n"


		"test_code:\n"

		//itp_loop for debug
		"nop;\n"
		"itp_loop_exit_code:\n"
		"mov eax, 6;\n"
		"nop;\n"
		"nop;\n"
		"nop;\n"
		"nop;\n"
		"cmp eax, 5;\n"
		"jz itp_loop_exit_code;\n"

		"test_code_cpuindex:\n"
		"mov edi, patch_str_core_id;\n" //populate core id statically

		//check static ring-0 virtual address for SECS for current view
		//verify secret value on page to ensure it is not malicious
		"test_code_secs_patch1:\n"
		"mov edx, patch_str_secs_ptr;\n"
		"mov eax, [edx+secs.scv];\n"
		"test_code_cmp_patch:\n"
		"cmp eax, patch_str_secs_scv;\n"
		"jz exit_code_cookie_check_ok;\n"

		// For view0, cookie value check will fail when running
		// in EPT environment because we do not reveal the secure
		// cookie value to the untrusted view.
		"mov ebx, [edx+secs.eid];\n"
		"cmp ebx, 0;\n"
		"jnz assert_invalid_secs;\n"

		//For unstrusted view, we compare the secs->scv with untrusted
		//scv
		"mov ebx, patch_str_secs_scv_un;\n"
		"cmp eax, ebx;\n"
		"jz exit_code_cookie_check_ok;\n"

		"assert_invalid_secs:\n"
		//VMCALL here to assert only for trusted view
		"mov eax, nr;\n"
		"mov ebx, service_type;\n"
		"mov ecx, vmcall_assert;\n"
		"vmcall;\n"

		"exit_code_cookie_check_ok:\n"
		//if 0 do not switch views, pass control to os handler
		"mov ax, [edx+secs.eid];\n"
		"cmp ax, 0;\n"
		"jz no_view_switch;\n"

		//we are in a trusted view

		//****************debug only start***************
		/*
		   "mov ebx, edx;\n"
		   "add ebx, secs.eid;\n"
		   "mov ax, 1;\n" //expect view 1
		   "mov cx, 0;\n" //switch eid to 0
		   ".byte 0xF0;\n"
		   "cmpxchg [ebx], cx;\n"
		 */
		//****************debug only end***************
		"mov eax, [esp+vector];\n"
		"cmp eax, exception_last;\n"
		"jg continue_aex_flow;\n"
		// Do not crash TA for nmi and mc exceptions
		"cmp eax, exception_nmi;\n"
		"je continue_aex_flow;\n"
		// This is an exception in TA - mark SECS.attributes.inited = 0
		"mov eax, [edx+secs.attrib];\n"
		"and eax, ~0x1;\n" //Clear secs.attributes.inited
		"mov [edx+secs.attrib], eax;\n"

		"continue_aex_flow:\n"
		//check boolean param whether to expect error code
		//on interrupt frame or not
		"mov eax, [esp+flag];\n"
		"cmp eax, 0;\n"
		"jz no_error_code;\n"

		//lookup CS for ring3 dpl and eip from intr frame
		//if not r3 CS - this should not happen since we should not be in
		//non-0 view and in ring-0 CS, unless, we have ring-0 views in which
		//case we will need to save state for ring-0 code via r0 TCS

		//skip top 36 bytes of stack frame since we get called from my_handler
		//which pushes eax, ebx, ecx, edx, esi, edi, vector,
		//errflag, retaddr (due to call)
		"mov eax, [esp+if_skip_ec+if.cs];\n" //cs
		"mov ebx, [esp+if_skip_ec+if.eip];\n" //interrupted eip
		"jmp continue_test;\n"

		"no_error_code:\n"
		"mov eax, [esp+if_noskip_ec+if.cs];\n" //cs
		"mov ebx, [esp+if_noskip_ec+if.eip];\n" //interrupted eip

		//we have cs and eip cached in eax and ebx resp.
		"continue_test:\n"
		//BUG: "mov eax, [esp+if_noskip_ec+if.cs];\n" //cs
		"and eax, ss_rpl_mask;\n"
		//BUG: "mov ebx, [esp+if_noskip_ec+if.eip];\n" //interrupted eip
		"cmp eax, ss_rpl_ring3;\n" //cmp with user/expected CS
		"jnz save_ret_noreplace;\n" //if not r3 CS dont touch intr frame

		/*
		//we have eip cached in ebx
		"mov eax, [edx+secs.base];\n" //get start of protected gla from secs
		"cmp ebx, eax;\n" //ensure EIP >= gla_start
		"jnae save_ret_noreplace;\n" //out_of_range if ebx !>= gla_start

		"add eax, [edx+secs.size];\n" //get start of protected gla from secs
		"cmp ebx, eax;\n" //ensure EIP <= gla_end
		"ja save_ret_noreplace;\n" //out of range if ebx > gla_end
		 */

		//we have to replace EIP on stack with AEP
		//and replace RSP on stack with external stack from TCS
		"jmp test_code_get_ssa;\n"

		"save_ret_noreplace:\n"

		"mov eax, [esp+flag];\n"
		"cmp eax, 0;\n"
		"jz get_ip_no_err;\n"

		"mov edx, [esp+if_skip_ec+if.eip];\n"
		"mov esi, [esp+if_noskip_ec+if.errcode];\n"
		"jmp go_vmcall_assert;\n"

		"get_ip_no_err:\n"
		"mov edx, [esp+if_noskip_ec+if.eip];\n"
		"mov esi, 0;\n"

		"go_vmcall_assert:\n"
		"mov eax, nr;\n"
		"mov ebx, service_type;\n"
		"mov ecx, vmcall_assert;\n"
		"vmcall;\n"

		"test_code_get_ssa:\n"
		//save original EIP (intr point) into SSA with RSP and other GPRS
		//all required gprs, esp, eip values should be on trusted stack

		//Establish ptr to TCS
		"mov ebx, edx;\n"
		"add ebx, secs.pcd;\n"
		"mov ecx, edi;\n" //we need to cache core id in edi
		"shl ecx, shift_size_secs_pcd;\n"
		"add ecx, ebx;\n" //ptr to core-specific pcd
		"mov ebx, [ecx+secs.pcd.tcs];\n"

		//Establish ptr to SSA frame (from TCS)
		//TCS is in ebx, OSSA, CSSA is a field in TCS struct
		//FIXME MAYBE - this could be done in EENTER, ERESUME
		//in which case this will require only one read from TCS
		"mov ecx, [ebx+tcs.ossa];\n"   //ossa
		"mov eax, [edx+secs.base];\n"  //base
		"add ecx, eax;\n"
		"mov eax, [ebx+tcs.cssa];\n"   //cssa
		"mov esi, [edx+secs.ssa_frame_size];\n" //ssa frame size
		"shl esi, shift_size_4k;\n" //ssa frame size is in pages
		"mul esi;\n" //size*cssa - result in edx:eax
		"add eax, ecx;\n" //add base FIXME LATER higher 32 bits edx ignored

		// XSAVE and clean
		//xsave registers on ssa starting from
		//the beginning.
		"fxsave [eax];\n"
		//Clear the fx registers by restoring value in them from
		//ssa  +512 byte offset.
		"fxrstor [eax+512];\n"
		//gpr area at end of ssa page
		"add eax, 4096;\n"
		"sub eax, ssa_gpr_size;\n"

		//ensure order accessed on stack is same as saved from my_handler
		"mov ecx, [esp+start_gprs+gprs.eax];\n" //save eax
		"mov [eax+ssa.ax], ecx;\n"
		"mov ecx, [esp+start_gprs+gprs.ecx];\n" //save ecx
		"mov [eax+ssa.cx], ecx;\n"
		"mov ecx, [esp+start_gprs+gprs.edx];\n" //save edx
		"mov [eax+ssa.dx], ecx;\n"
		"mov ecx, [esp+start_gprs+gprs.ebx];\n" //save ebx
		"mov [eax+ssa.bx], ecx;\n"
		"mov ecx, [esp+start_gprs+gprs.esi];\n" //save esi
		"mov [eax+ssa.si], ecx;\n"
		"mov ecx, [esp+start_gprs+gprs.edi];\n" //save edi
		"mov [eax+ssa.di], ecx;\n"

		"mov [eax+ssa.bp], ebp;\n"    //save ebp directly since we dont clobber
		//no r8-r15

		//Again, check boolean flag param whether to expect error code
		//on interrupt frame or not
		"mov ecx, [esp+flag];\n"
		"cmp ecx, 0;\n"
		"jz save_eflags_no_errcode;\n"

		"mov ecx, [esp+if_skip_ec+if.eflags];\n" //save eflags after skipping error code
		"mov [eax+ssa.flags], ecx;\n"     //FIXME LATER need to scrub eflags right

		"mov ecx, [esp+if_skip_ec+if.eip];\n" //save eip
		"mov [eax+ssa.ip], ecx;\n"
		"mov ecx, [ebx+tcs.aep];\n"
		"mov [esp+if_skip_ec+if.eip], ecx;\n" //replace eip with aep

		"mov ecx, [esp+if_skip_ec+if.esp];\n" //save esp
		"mov [eax+ssa.sp], ecx;\n"
		"mov ecx, [eax+ssa.sp_u];\n"
		"mov [esp+if_skip_ec+if.esp], ecx;\n" //replace esp with u_esp
		"jmp continue_ssa_save;\n"

		"save_eflags_no_errcode:\n"
		"mov ecx, [esp+if_noskip_ec+if.eflags];\n" //save eflags - no error code
		"mov [eax+ssa.flags], ecx;\n"

		"mov ecx, [esp+if_noskip_ec+if.eip];\n" //save eip
		"mov [eax+ssa.ip], ecx;\n"
		"mov ecx, [ebx+tcs.aep];\n"
		"mov [esp+if_noskip_ec+if.eip], ecx;\n" //replace eip with aep

		"mov ecx, [esp+if_noskip_ec+if.esp];\n" //save esp
		"mov [eax+ssa.sp], ecx;\n"
		"mov ecx, [eax+ssa.sp_u];\n"
		"mov [esp+if_noskip_ec+if.esp], ecx;\n" //replace esp with u_esp

		"continue_ssa_save:\n"
		//FIXME LATER - update exit_info struct at offset 160

		//update CSSA in TCS
		"mov ecx, [ebx+tcs.cssa];\n" //cssa
		"inc ecx;\n"
		"mov [ebx+tcs.cssa], ecx;\n"


		//put synthetic state on stack for ESP=ESP_U, EBP=EBP_U and EIP=AEP_U
		//SWITCH stacks to RSP_U and copy interupt frame on RSP_u from RSP_t
		//add synthetic state for GPRs and add 8 for vector and flag so that
		//RSP_u is setup such that whether view switch happens or not
		//code following no_view_switch label always does the same thing:
		//pops passed params, and transfers control to my_handler which
		//pops synthetic (or real values of GPRs) and rets to OS ISR via edx

		/*
		//bug no need to modify r0 stack - we have already modified esp on stack
		//we still need to scrub registers as done below...
		"mov esi, esp;\n"
		"mov esp, [eax+ssa.sp_u];\n" //[eax+144] references esp_u
		 */
		"mov ebp, [eax+ssa.bp_u];\n" //[eax+152] references ebp_u

		//-----No More trusted stack accesses after this point---------

		//need to setup u_stack frame the same way as non_vs exit
		//push interrupt frame - aep and rsp have already been updated on frame
		//push synthetic gprs which will be popped by my_handler code
		//push dummy vector, flag, my_handler offset after call to this function
		//ebx has ptr to trusted esp
		"mov ecx, eresume_leaf;\n"
		"mov [esp+start_gprs+gprs.eax], ecx;\n" //eax=eresume leaf
		"mov ecx, [ebx+tcs.aep];\n"
		"mov [esp+start_gprs+gprs.ecx], ecx;\n" //ecx=AEP
		"mov ecx, 0;\n"
		"mov [esp+start_gprs+gprs.edx], ecx;\n" //copy 0 edx
		"mov ecx, ebx;\n"
		"mov [esp+start_gprs+gprs.ebx], ecx;\n" //ebx=TCS
		"mov ecx, 0;\n"
		// Check if the vector indicates an exception.
		// if (vector == exception)
		// Copy error code in gprs.esi
		// Copy faulting IP in gprs.edi
		// else
		// copy 0 to esi and 0 to edi
		"mov [esp+start_gprs+gprs.esi], ecx;\n" //copy 0 esi
		"mov [esp+start_gprs+gprs.edi], ecx;\n" //copy 0 edi
		"mov esi, [esp+vector];\n"
		"cmp esi, exception_last;\n"
		"jg no_trusted_view_exception;\n"

		"cmp esi, exception_nmi;\n"
		"je no_trusted_view_exception;\n"

		"mov ecx, enclave_crashed;\n"
		"mov [esp+start_gprs+gprs.esi], ecx;\n" //copy error code to  esi
		"mov ecx, [eax+ssa.ip];\n"
		"mov [esp+start_gprs+gprs.edi], ecx;\n" //copy faulting address to edi
		"mov [esp+start_gprs+gprs.edx], esi;\n" // Copy error code to  edx
		"no_trusted_view_exception:\n"

		//no r8-r15

		//STACKFIX Ravi - at this point the stack frame is ready to be used
		//### get address of exit page
		"test_code_exit_page_patch:\n"
		"mov eax, patch_str_exit_page;\n"
		"mov ecx, eax;\n"
		//### bulk copy stack frame from trusted r0 stack to exit-page <rw>
		"pop eax;\n" //caller rip
		"mov [ecx], eax;\n"
		"pop eax;\n" //flag
		"mov [ecx+flag], eax;\n"
		"pop eax;\n" //vector
		"mov [ecx+vector], eax;\n"
		"pop eax;\n" //eax
		"mov [ecx+start_gprs+gprs.eax], eax;\n"
		"pop eax;\n" //ecx
		"mov [ecx+start_gprs+gprs.ecx], eax;\n"
		"pop eax;\n" //edx
		"mov [ecx+start_gprs+gprs.edx], eax;\n"
		"pop eax;\n" //ebx
		"mov [ecx+start_gprs+gprs.ebx], eax;\n"
		"pop eax;\n" //esi
		"mov [ecx+start_gprs+gprs.esi], eax;\n"
		"pop eax;\n" //edi
		"mov [ecx+start_gprs+gprs.edi], eax;\n"
		//### check for err code
		"mov eax, [ecx+flag];\n"
		"cmp eax, 0;\n"
		"jz no_err_code_to_copy;\n"

		"pop eax;\n" //errcode
		"mov [ecx+if_noskip_ec+if.errcode], eax;\n"
		"pop eax;\n"//eip
		"mov [ecx+if_skip_ec+if.eip], eax;\n" //eip
		"pop eax;\n" //cs
		"mov [ecx+if_skip_ec+if.cs], eax;\n"
		"pop eax;\n" //rflags
		"mov [ecx+if_skip_ec+if.eflags], eax;\n"
		"pop eax;\n" //rsp
		"mov [ecx+if_skip_ec+if.esp], eax;\n"
		"pop eax;\n" //ss
		"mov [ecx+if_skip_ec+if.ss], eax;\n"
		"jmp continue_with_copy;\n"

		"no_err_code_to_copy:\n"
		"pop eax;\n" //eip
		"mov [ecx+if_noskip_ec+if.eip], eax;\n"
		"pop eax;\n" //cs
		"mov [ecx+if_noskip_ec+if.cs], eax;\n"
		"pop eax;\n" //eflags
		"mov [ecx+if_noskip_ec+if.eflags], eax;\n"
		"pop eax;\n" //esp
		"mov [ecx+if_noskip_ec+if.esp], eax;\n"
		"pop eax;\n" //ss
		"mov [ecx+if_noskip_ec+if.ss], eax;\n"

		"continue_with_copy:\n"
		//#### adjust untrusted rsp to remove frame - already done above in pops
		//#### (do we need to - will always be used from tss->rsp0)
		//#### cache untrusted r0 stack ptr from TCS?? into r14
		"mov eax, [ebx+tcs.ur0sp];\n"
		"mov [ecx+ur0sp], eax;\n" //Save untrusted r0sp on exit page
		"push ecx;\n" // Temporarily push exit page on trusted stack
		//Pop it in a GPR before switching to the untrusted view
		/*
		//no need to do all this - r0 stack has all info already
		"mov ecx, [esi+flag];\n"     //read flag
		"mov [esp+flag], ecx;\n"     //copy flag
		"mov ecx, [esi+vector];\n"     //read vector
		"mov [esp+vector], ecx;\n"     //copy vector
		//intr frame
		"mov ecx, [esi+flag];\n"   //what was the real err flag
		"cmp ecx, 0;\n"
		"jz save_u_stk_no_errcode;\n"

		"mov ecx, [esi+if_noskip_ec+if.errcode];\n" //save error code
		"mov [esp+if_noskip_ec+if.errcode], ecx;\n"
		"mov ecx, [esi+if_skip_ec+if.eflags];\n" //save eflags
		"mov [esp+if_skip_ec+if.eflags], ecx;\n"
		"mov ecx, [esi+if_skip_ec+if.eip];\n" //save eip
		"mov [esp+if_skip_ec+if.eip], ecx;\n"
		"mov ecx, [esi+if_skip_ec+if.esp];\n"
		"mov [esp+if_skip_ec+if.esp], ecx;\n" //save u_esp

		"save_u_stk_no_errcode:\n"
		"mov ecx, [esi+if_noskip_ec+if.eflags];\n" //save eflags - no error code
		"mov [esp+if_noskip_ec+if.eflags], ecx;\n"
		"mov ecx, [esp+if_noskip_ec+if.eip];\n" //save eip
		"mov [esp+if_noskip_ec+if.eip], ecx;\n"
		"mov ecx, [esi+if_noskip_ec+if.esp];\n"
		"mov [esp+if_noskip_ec+if.esp], ecx;\n" //save u_esp

		//copy real isr exit point
		"mov ecx, [esi];\n"
		"mov [esp], ecx;\n"
		 */

		//Make TCS state inactive
		//lock cmpxchg tcs.state from expect_active to  set_inactive
		"mov eax, tcs_state_expect_active;\n"
		"mov ecx, tcs_state_set_inactive;\n"
		".byte 0xF0;\n"
		"cmpxchg [ebx], ecx;\n"
		"je tcs_state_inactive_ok;\n"

		"mov eax, nr;\n"
		"mov ebx, service_type;\n"
		"mov ecx, vmcall_assert;\n"
		"vmcall;\n"

		"tcs_state_inactive_ok:\n"
		//we need to save the FS, GS base and limit on EENTER into TCS
		//we access gdt and ldt from a SECS cached location cached during creation
		//so that we dont have to perform SGDT again (also exit will occur)
		//FIXME LATER - assumes FS, GS is always pointing to GDT and not LDT

		//edi has core id cached, esi is available
		"test_code_secs_patch2:\n"
		"mov edx, patch_str_secs_ptr;\n" //since edx was clobbered in the mul above
		"mov esi, edx;\n"
		"add esi, secs.pcd;\n"
		"mov ecx, edi;\n" //we need to cache core id in edi
		"shl ecx, shift_size_secs_pcd;\n"
		"add esi, ecx;\n" //get ptr to core-specific pcd

		//swap fs
		"mov eax, [esi+secs.pcd.gdtr];\n"
		"xor ecx, ecx;\n"
		"mov cx, [ebx+tcs.save_fs_selector];\n"
		"mov edx, ecx;\n"
		"and ecx, 0xFFF8;\n" //shift TI and RPL fields out and mul by 8
		"and edx, 4;\n" //fs.TI
		"cmp edx, 0;\n"
		"je  cont_fs_restore_aex;\n"
		"mov eax, [esi+secs.pcd.ldtr];\n" //restore in LDT

		"cont_fs_restore_aex:\n"
		"add eax, ecx;\n"
		"mov ecx, [ebx+tcs.save_fs_desc_low];\n"
		"mov [eax], ecx;\n"
		"mov ecx, [ebx+tcs.save_fs_desc_high];\n"
		"mov [eax+4], ecx;\n"

		"xor ecx, ecx;\n"
		"mov cx, [ebx+tcs.save_fs_selector];\n"
		"mov fs, ecx;\n"

		//swap gs
		"mov eax, [esi+secs.pcd.gdtr];\n"
		"xor ecx, ecx;\n"
		"mov cx, [ebx+tcs.save_gs_selector];\n"
		"mov edx, ecx;\n"
		"shr ecx, 3;\n" //shift TI and RPL fields out
		"shl ecx, 3;\n" //selector x 8 (bytes)
		"and edx, 4;\n" //gs.TI
		"cmp edx, 0;\n"
		"je  cont_gs_restore_aex;\n"
		"mov eax, [esi+secs.pcd.ldtr];\n" //restore GS in LDT

		"cont_gs_restore_aex:\n"
		"add eax, ecx;\n"
		"mov ecx, [ebx+tcs.save_gs_desc_low];\n"
		"mov [eax], ecx;\n"
		"mov ecx, [ebx+tcs.save_gs_desc_high];\n"
		"mov [eax+4], ecx;\n"

		"xor ecx, ecx;\n"
		"mov cx, [ebx+tcs.save_gs_selector];\n"
		"mov gs, ecx;\n"
		//-----No More TCS accesses after this point---------

		"pop edx;\n" //exit page address - preserve till copy from exit page to untrusted stack
#ifdef ACTIVATE_ACR3
		//switch CR3 to CR3_u from TCS
		"mov eax, [ebx+tcs.os_cr3];\n"
		"mov cr3, eax;\n"
#endif
		//vmfunc to view 0 always (exit) - this could be on a seperate page

		"mov eax, vmfunc_view_sw_ctrl;\n"
		"mov ecx, untrusted_view_id;\n"
		//"vmcall;\n" //VMCALL only for debug
		//VMFUNC emitted below
		".byte 0x0f;\n"
		".byte 0x01;\n"
		".byte 0xd4;\n"

		//STACKFIX Ravi - at this point stack frame is copied to exit-page edx <ro>
		//### copy stack frame from exit-page to untrusted r0 stack [edx+ur0sp])
		//### alternately tss has remapped to os original so we could also read tss->esp0 to use
		//### pivot stack to untrusted stack cached in r14
		"mov eax, [edx+ur0sp];\n" //Untrusted r0 stack
		"mov esp, eax;\n" // Switch to untrusted ring0 stack
		//### check for err code
		"mov eax, [edx+flag];\n"
		"cmp eax, 0;\n"
		"jz no_err_code_to_copy_2;\n"

		"mov eax, [edx+if_skip_ec+if.ss];\n"
		"push eax;\n" //ss
		"mov eax, [edx+if_skip_ec+if.esp];\n"
		"push eax;\n" //rsp
		"mov eax, [edx+if_skip_ec+if.eflags];\n"
		"push eax;\n" //rflags
		"mov eax, [edx+if_skip_ec+if.cs];\n"
		"push eax;\n" //cs
		"mov eax, [edx+if_skip_ec+if.eip];\n" //eip
		"push eax;\n"//eip
		"mov eax, [edx+if_noskip_ec+if.errcode];\n"
		"push eax;\n" //errcode

		"jmp continue_with_copy_2;\n"
		"no_err_code_to_copy_2:\n"
		"mov eax, [edx+if_noskip_ec+if.ss];\n"
		"push eax;\n" //ss
		"mov eax, [edx+if_noskip_ec+if.esp];\n"
		"push eax;\n" //rsp
		"mov eax, [edx+if_noskip_ec+if.eflags];\n"
		"push eax;\n" //rflags
		"mov eax, [edx+if_noskip_ec+if.cs];\n"
		"push eax;\n" //cs
		"mov eax, [edx+if_noskip_ec+if.eip];\n" //eip
		"push eax;\n"//eip

		"continue_with_copy_2:\n"
		"mov eax, [edx+start_gprs+gprs.edi];\n"
		"push eax;\n" //edi
		"mov eax, [edx+start_gprs+gprs.esi];\n"
		"push eax;\n" //esi
		"mov eax, [edx+start_gprs+gprs.ebx];\n"
		"push eax;\n" //ebx
		"mov eax, [edx+start_gprs+gprs.edx];\n"
		"push eax;\n" //edx
		"mov eax, [edx+start_gprs+gprs.ecx];\n"
		"push eax;\n" //ecx
		"mov eax, [edx+start_gprs+gprs.eax];\n"
		"push eax;\n" //eax
		"mov eax, [edx+vector];\n"
		"push eax;\n" //vector
		"mov eax, [edx+flag];\n"
		"push eax;\n" //flag
		"mov eax, [edx];\n"
		"push eax;\n" //caller rip
		//if (vector == to_be_handled_exception)
		//else, jmp no_view_switch
		"mov esi, [esp+vector];\n"
		"cmp esi, exception_last;\n"
		"jg after_view_switch;\n"

		// No TA exception handling
		"cmp esi, exception_nmi;\n"
		"je no_view_switch;\n"
		"cmp esi, exception_mc;\n"
		"je after_view_switch;\n"

		"jmp exit_ta_exception;\n"

		"after_view_switch:\n"
		"no_view_switch:\n" //if no view switch, no cr3 or stack switch either

		//get redirect base address for this core
		"test_code_ptr_core_state_patch:\n"
		"mov ebx, patch_str_core_state_ptr;\n"
		"mov eax, edi;\n"   //patch core-id during vidt setup
		"mov  ecx, size_core_state;\n"
		"mul ecx;\n" // *** edx <- high order bits, but multiplication is small. ***
		"add ebx, eax;\n" //get ptr to per core state - retain in ebx
		//fetch OS ISR address and cache in edx
		"mov eax, [ebx + redirect_table];\n" //get redirect base address for this core
		"mov ecx, [esp + vector];\n"  //get int vector from prepped u_stack (esp+8)
		"mov edi, [eax + ecx * sizeof_redirect_entry];\n" //edi = os isr address

		"mov eax, 1;\n" //to indicate to my_handler to use ret
		"ret params_size;\n"
		//pop errflag and vector passed by my_handler entry code
		//NOTE edi has the os original handler address
		//my_handler will xchg edi with TOS and ret to kernel ISR cleanly

		//Ravi - keep this block - it us unused for now
		//Ravi - it is used to check result after pf to add page to acr3
		//special exit to not touch my_handler ISR - to be able
		//to conditionally pop error code for events like PF!!
		"exit_ta_exception:\n"
		"pop eax;" //pop return addr for my_handler!!
		"pop eax;" //pop flag
		"cmp eax, 0;\n"
		"je exit_no_error_code_pop;\n"
		"pop eax;" //pop vector
		"pop eax;\n"
		"pop ecx;\n"
		"pop edx;\n"
		"pop ebx;\n"
		"pop esi;\n"
		"pop edi;\n"

		"add esp, 4;\n" //pop error code without killing gprs!
		"iret;\n"

		"exit_no_error_code_pop:\n"
		"pop eax;" //pop vector
		"pop eax;\n"
		"pop ecx;\n"
		"pop edx;\n"
		"pop ebx;\n"
		"pop esi;\n"
		"pop edi;\n"

		"iret;\n"
		".att_syntax\n"
);
void test_code_end(void);
__asm__ (
		".globl test_code_end\n"
		"test_code_end:"

		"nop;\n"
	);

//EEXIT VIEW FLOW
//NOTE - expected to be called only from my_handler code
//Expects to get a flag param on stack to skip error code
//based on IDT vector - some entries below 20 have error codei
//This exit flow is performed through a ring-0 trampoline memory pages
//referenced via the asserted page table setup for the Trusted View by
//the initialization flow. This flow is initiated by software executing
//inside the TV using an INT opcode that transfers control to the
//trusted ring-0 trampoline code.
//Pre-conditions:
//Executing inside in trusted view
//GPRs passed in are the same as EEXIT (RAX (in) 04h, RBX (in) holds
//the address to branch outside the trusted view, RCX (out) returns AEP)
//Note - Responsibility of tRTS software to clear out GPR state and swap
//stack to OS-external stack from TCS and then invoke this flow via INT

void exit_code_cpuindex(void);
void exit_code_cmp_patch(void);
void exit_code_secs_patch1(void);
void exit_code_exit_page_patch(void);
void exit_code(void);
__asm__ (
		".intel_syntax noprefix\n"
		".globl exit_code_cpuindex\n"
		".globl exit_code\n"
		".globl exit_code_cmp_patch\n"
		".globl exit_code_secs_patch1\n"
		".globl exit_code_exit_page_patch\n"

		//NOTE WELL - this routine uses all the defines from
		//test_code (async exit flow)

		"exit_code:\n"

		//itp_loop for debug
		"nop\n"
		"exit_code_itp_loop:\n"
		"mov eax, 3;\n"
		"nop;\n"
		"nop;\n"
		"nop;\n"
		"nop;\n"
		"cmp eax, 2;\n"
		"jz exit_code_itp_loop;\n"

		"exit_code_step_1:\n"
		//check static ring-0 virtual address for SECS for current view
		//verify secret value on page to ensure it is not malicious
		"exit_code_secs_patch1:\n"
		"mov edx, patch_str_secs_ptr;\n"
		"mov eax, [edx+secs.scv];\n"
		"exit_code_cmp_patch:\n"
		"cmp eax, patch_str_secs_scv;\n"
		"jz exit_code_step_2;\n"

		//VMCALL here to assert
		"mov eax, nr;\n"
		"mov ebx, service_type;\n"
		"mov ecx, vmcall_assert;\n"
		"vmcall;\n"

		"exit_code_step_2:\n"
		//if 0 then assert
		"mov ax, [edx+secs.eid];\n"
		"cmp ax, 0;\n"
		"jne exit_code_step_3;\n"

		//VMCALL here to assert
		"mov eax, nr;\n"
		"mov ebx, service_type;\n"
		"mov ecx, vmcall_assert;\n"
		"vmcall;\n"

		//we are in a trusted view

		"exit_code_step_3:\n"
		//****************debug only start***************
		/*
		   "mov ebx, edx;\n"
		   "add ebx, secs.eid;\n"
		   "mov ax, 1;\n" //expect view 1
		   "mov cx, 0;\n" //switch eid to 0
		   ".byte 0xF0;\n"
		   "cmpxchg [ebx], cx;\n"
		 */
		//****************debug only end***************

		//ignore boolean param whether to expect error code
		//on interrupt frame or not - since we know this is a sw int flow

		//lookup CS for ring3 dpl and eip from intr frame
		//if not r3 CS - this should not happen since we should not be in
		//non-0 view and in ring-0 CS, unless, we have ring-0 views
		"mov eax, [esp+if_noskip_ec+if.cs];\n" //cs
		"and eax, ss_rpl_mask;\n"
		"mov ebx, [esp+if_noskip_ec+if.eip];\n" //interrupted eip
		"cmp eax, ss_rpl_ring3;\n" //cmp with user/expected CS
		"jz exit_code_step_4;\n" //if not r3 CS dont touch intr frame

		//and IP <= enclave.base + enclave.size
		//"check_r0_rpl:\n"
		//"cmp eax, ss_rpl_ring0;\n"
		//"jz exit_code_step_5;\n"
		//"mov eax, nr;\n"
		//"mov ebx, service_type;\n"
		//"mov ecx, vmcall_assert;\n"
		//"mov edx, 0xdf;\n"
		//"mov esi, 0xdf;\n"
		//"vmcall;\n"

		//TODO: below checks on EIP can be enabled for KTA also
		//TODO: Will enable once Vasu changes the enclave base to 0
		"exit_code_step_4:\n"
		//we have eip cached in ebx
		"mov eax, [edx+secs.base];\n" //get start of protected gla from secs
		"cmp ebx, eax;\n" //ensure EIP >= gla_start
		"jae exit_code_step_4b;\n"

		"mov eax, nr;\n"
		"mov ebx, service_type;\n"
		"mov ecx, vmcall_assert;\n"
		"vmcall;\n"

		"exit_code_step_4b:\n"
		"add eax, [edx+secs.size];\n" //get start of protected gla from secs
		"cmp ebx, eax;\n" //ensure EIP <= gla_end
		"jbe exit_code_step_5;\n"

		"mov eax, nr;\n"
		"mov ebx, service_type;\n"
		"mov ecx, vmcall_assert;\n"
		"vmcall;\n"

		"exit_code_step_5:\n"
		//we have to replace EIP on stack with RBX (desired target)

		//Establish ptr to TCS to read AEP (we need to pass it back in RCX)
		"mov ebx, edx;\n"
		"add ebx, secs.pcd;\n"
		"exit_code_cpuindex:\n"
		"mov edi, patch_str_core_id;\n" //populate core id statically
		"shl edi, shift_size_secs_pcd;\n"
		"add edi, ebx;\n" //ptr to core-specific pcd
		"mov ebx, [edi+secs.pcd.tcs];\n"

		//setup RCX to contain AEP, fills ecx on untrusted stack setup
		//by my_handler
		"mov ecx, [ebx+tcs.aep];\n"
		"mov [esp+start_gprs+gprs.ecx], ecx;\n" //ecx=AEP

		//cache os-cr3 from tcs into gpr FIXME LATER
		"mov esi, [ebx+tcs.os_cr3];\n"
		//"mov eax, [esp+vector];\n"
		//"cmp eax, kexit_vector;\n"

		//TODO: This will also be re-enabled
		//Make TCS state inactive
		//lock cmpxchg tcs.state from expect_active to  set_inactive
		//RAHIL: Done
		"mov eax, tcs_state_expect_active;\n"
		"mov ecx, tcs_state_set_inactive;\n"
		".byte 0xF0;\n"
		"cmpxchg [ebx], ecx;\n"
		"je exit_code_step_6;\n"

		"mov eax, nr;\n"
		"mov ebx, service_type;\n"
		"mov ecx, vmcall_assert;\n"
		"vmcall;\n"

		"exit_code_step_6:\n"
		"mov eax, [esp+if_noskip_ec+if.cs];\n" //cs
		"and eax, ss_rpl_mask;\n"
		"cmp eax, ss_rpl_ring0;\n"
		"jz set_exit_point;\n"

		//TODO: This can be removed for KTA
		//RAHIL: Done
		//we need to save the FS, GS base and limit on EENTER into TCS
		//we access gdt and ldt from a SECS cached location cached during creation
		//so that we dont have to perform SGDT again (also exit will occur)
		//FIXME LATER - assumes FS, GS is always pointing to GDT and not LDT

		//swap fs
		//edi has core-specific pcd
		"mov eax, [edi+secs.pcd.gdtr];\n" //gdtr in eax
		"xor ecx, ecx;\n"
		"mov cx, [ebx+tcs.save_fs_selector];\n"
		"mov edx, ecx;\n"
		"and ecx, 0xFFF8;\n" //shift TI and RPL fields out and mul by 8
		"and edx, 4;\n" //fs.TI
		"cmp edx, 0;\n"
		"je cont_fs_restore_exit;\n"
		"mov eax,[edi+secs.pcd.ldtr];\n" //restore in LDT

		"cont_fs_restore_exit:\n"
		"add eax, ecx;\n"
		"mov ecx, [ebx+tcs.save_fs_desc_low];\n"
		"mov [eax], ecx;\n"
		"mov ecx, [ebx+tcs.save_fs_desc_high];\n"
		"mov [eax+4], ecx;\n"

		"xor ecx, ecx;\n"
		"mov cx, [ebx+tcs.save_fs_selector];\n"
		"mov fs, cx;\n"

		//swap gs
		"mov eax, [edi+secs.pcd.gdtr];\n" //gdtr in eax
		"xor ecx, ecx;\n"
		"mov cx, [ebx+tcs.save_gs_selector];\n"
		"mov edx, ecx;\n"
		"shr ecx, 3;\n" //shift TI and RPL fields out
		"shl ecx, 3;\n" //selector x 8 (bytes)
		"and edx, 4;\n" //gs.TI
		"cmp edx, 0;\n"
		"je cont_gs_restore_exit;\n"
		"mov eax,[edi+secs.pcd.ldtr];\n" //restore gs in LDT

		"cont_gs_restore_exit:\n"
		"add eax, ecx;\n"
		"mov ecx, [ebx+tcs.save_gs_desc_low];\n"
		"mov [eax], ecx;\n"
		"mov ecx, [ebx+tcs.save_gs_desc_high];\n"
		"mov [eax+4], ecx;\n"

		"xor ecx, ecx;\n"
		"mov cx, [ebx+tcs.save_gs_selector];\n"
		"mov gs, ecx;\n"

		//set desired target for iret
		"set_exit_point:\n"
		"mov ecx, [esp+start_gprs+gprs.ebx];\n" //tRTS passed desired target in ebx
		"mov [esp+if_noskip_ec+if.eip], ecx;\n" //replace eip with desired target
		//Restore EFLAGS
		"mov ecx, [ebx+tcs.eflags];\n"
		"mov [esp+if_noskip_ec+if.eflags], ecx;\n"

		//rsp on stack should already by untrusted stack (switched by tRTS)
		//STACKFIX Ravi - we need to move the bulk frame to exit-page
		//### get location of exit-page <rw>
		"exit_code_exit_page_patch:\n"
		"mov eax, patch_str_exit_page;\n"
		"mov edx, eax;\n"
		//### bulk copy stack frame from trusted r0 stack to exit-page <rw>
		"pop eax;\n" //caller rip
		"mov [edx], eax;\n"
		"pop eax;\n" //error flag
		"mov [edx+flag], eax;\n"
		"pop eax;\n" //vector
		"mov edi, eax;\n" //edi == vector
		"mov [edx+vector], eax;\n"
		"pop eax;\n" //eax
		"mov [edx+start_gprs+gprs.eax], eax;\n"
		"pop eax;\n" //rcx
		"mov [edx+start_gprs+gprs.ecx], eax;\n"
		"pop eax;\n" //rdx
		"mov [edx+start_gprs+gprs.edx], eax;\n"
		"pop eax;\n" //rbx
		"mov [edx+start_gprs+gprs.ebx], eax;\n"
		"pop eax;\n" //rsi
		"mov [edx+start_gprs+gprs.esi], eax;\n"
		"pop eax;\n" //rdi
		"mov [edx+start_gprs+gprs.edi], eax;\n"
		"pop eax;\n" //eip
		"mov [edx+if_noskip_ec+if.eip], eax;\n"
		"pop eax;\n" //cs
		"mov [edx+if_noskip_ec+if.cs], eax;\n"
		"pop eax;\n" //rflags
		"mov [edx+if_noskip_ec+if.eflags], eax;\n"
		"mov eax, [edx+if_noskip_ec+if.cs];\n" //cs
		"and eax, ss_rpl_mask;\n"
		"cmp eax, ss_rpl_ring0;\n"
		//  "cmp edi, kexit_vector;\n"
		"jz save_ur0sp;\n"
		"pop eax;\n" //rsp
		"mov [edx+if_noskip_ec+if.esp], eax;\n"
		"pop eax;\n" //ss
		"mov [edx+if_noskip_ec+if.ss], eax;\n"
		"save_ur0sp:\n"
		//### adjust trusted r0 stack ptr - done with pops
		//### (not needed since will be used from tss automatically)
		//### cache untrusted r0 stack ptr from TCS into [edx+ur0sp]
		"mov eax, [ebx+tcs.ur0sp];\n"
		"mov [edx+ur0sp], eax;\n"


		//vmfunc to view 0 always (exit) - this could be on a seperate page
		"mov eax, vmfunc_view_sw_ctrl;\n"
		"mov ecx, untrusted_view_id;\n"
		#ifdef ACTIVATE_ACR3
		//switch cr3 to value cached in gpr - note - imp to do this before
		//view switched - since acr3 is only mapped in trusted view so
		//if you switch cr3 afterwards, that code will never run after vmfunc
		//to untrusted view (since your cr3 is invalid!)
		"mov cr3, esi;\n"
		#endif
		//TODO: Do this before vmfunc
		//RAHIL: Done
		"mov esp, [edx+ur0sp];\n"
		//"vmcall;\n" //VMCALL only for debug
		//VMFUNC opcode emitted below
		".byte 0x0f;\n"
		".byte 0x01;\n"
		".byte 0xd4;\n"

		//STACKFIX Ravi - at this point stack frame is copied to exit-page edx <ro>
		//### copy stack frame from exit-page to untrusted r0 stack [edx+ur0sp]
		//### alternately tss has remapped to os original so we could also read tss->esp0 to use
		//### pivot stack to untrusted stack cached in r14

		"mov eax, [edx+if_noskip_ec+if.cs];\n" //cs
		"and eax, ss_rpl_mask;\n"
		"cmp eax, ss_rpl_ring0;\n"
		// "mov eax, [edx+vector];\n"
		// "cmp eax, kexit_vector;\n"
		"jz cont_copy_from_exit_page;\n" //SS and ESP donot get pushed on intra-privilege call.

		"mov eax, [edx+if_noskip_ec+if.ss];\n"
		"push eax;\n" //ss
		"mov eax, [edx+if_noskip_ec+if.esp];\n"
		"push eax;\n" //rsp

		"cont_copy_from_exit_page:\n"
		"mov eax, [edx+if_noskip_ec+if.eflags];\n"
		"push eax;\n" //rflags
		"mov eax, [edx+if_noskip_ec+if.cs];\n"
		"push eax;\n" //cs
		"mov eax, [edx+if_noskip_ec+if.eip];\n" //eip
		"push eax;\n"//eip
		"mov eax, [edx+start_gprs+gprs.edi];\n"
		"push eax;\n" //rdi
		"mov eax, [edx+start_gprs+gprs.esi];\n"
		"push eax;\n" //rsi
		"mov eax, [edx+start_gprs+gprs.ebx];\n"
		"push eax;\n" //rbx
		"mov eax, [edx+start_gprs+gprs.edx];\n"
		"push eax;\n" //rdx
		"mov eax, [edx+start_gprs+gprs.ecx];\n"
		"push eax;\n" //rcx
		"mov eax, [edx+start_gprs+gprs.eax];\n"
		"push eax;\n" //eax
		"mov eax, [edx+vector];\n"
		"push eax;\n" //vector
		"mov eax, [edx+flag];\n"
		"push eax;\n" //flag
		"mov eax, [edx];\n"
		"push eax;\n" //caller rip
		"mov eax, 0;\n" //return flag to indicate use IRET not RET
		"ret params_size;\n"
		//We return a flag in eax to my_handler so that it checks that
		//and either does a RET to the OS handler (like AEX) to address in edi OR
		//IRETs for our INT flows, in either case, my_handler leaves only the
		//required intr frame on the appropriate stack before issueing IRET or RET
		//IRET cleans up the intr frame, and RET does not (as intended)
		".att_syntax\n"
);
void exit_code_end(void);
__asm__ (
		".globl exit_code_end\n"
		"exit_code_end:"
		"nop;\n"
	);


// this is a "function" that tests if the base of segment whose
// selector is in eax is 0; it assumes gdt is in esi and ldt in edi
// it leaves copies of the segment.lo in edx and hi in ecxi
#define CHECK_SEGMENT_BASE(seg) \
	"mov edx, eax;\n" /*make a copy*/ \
	"shr eax, 3;\n" /*eax has index*/ \
	"and edx, 4;\n" /*edx has table type*/ \
	\
	"cmp edx, 0;\n" /*if GDT*/ \
	"jne use_ldt_"seg";\n" \
	"mov ecx, [esi+eax*8];\n" \
	"mov eax, [esi+eax*8+4];\n" \
	"jmp dt_used_"seg";\n" \
	"use_ldt_"seg":\n" \
	"mov ecx, [edi+eax*8];\n" \
	"mov eax, [edi+eax*8+4];\n" \
	"dt_used_"seg":\n" \
	\
	"mov edx, ecx;\n" /*make a copy we need it later*/ \
	"and ecx, 0xFFFF0000;\n" /*now check the 3 base fields*/ \
	"cmp ecx, 0;\n" /*ecx has lo 32 bits*/ \
	"jz base_address_15_00_check_ok_"seg";\n" \
	"jmp gp_vmcall;\n" \
	\
	"base_address_15_00_check_ok_"seg":\n" \
	"mov ecx, eax;\n" /*eax has hi 32 bits*/ \
	"and eax, 0xFF0000FF;\n" \
	"cmp eax, 0;\n" \
	"jz base_address_31_16_check_ok_"seg";\n" \
	\
	"jmp gp_vmcall;\n" \
	\
	"base_address_31_16_check_ok_"seg":\n" \
	"nop;\n" \
	"nop;\n"


void enter_eresume_code_cpuindex(void);
void enter_eresume_code_cmp_patch1(void);
void enter_eresume_code_cmp_patch2(void);
void enter_eresume_code_secs_patch1(void);
void enter_eresume_code_secs_patch2(void);
void enter_eresume_code_secs_patch3(void);
void enter_eresume_code_secs_patch4(void);
void enter_eresume_code_secs_patch5(void);
void enter_eresume_code_secs_patch6(void);
void enter_eresume_code_enter_page_patch(void);
void enter_eresume_code(void);
__asm__ (
		".intel_syntax noprefix\n"
		".globl enter_eresume_code_cpuindex\n"
		".globl enter_eresume_code\n"
		".globl enter_eresume_code_secs_patch1\n"
		".globl enter_eresume_code_secs_patch2\n"
		".globl enter_eresume_code_secs_patch1\n"
		".globl enter_eresume_code_secs_patch2\n"
		".globl enter_eresume_code_secs_patch3\n"
		".globl enter_eresume_code_secs_patch4\n"
		".globl enter_eresume_code_secs_patch5\n"
		".globl enter_eresume_code_secs_patch6\n"
		".globl enter_eresume_code_cmp_patch1\n"
		".globl enter_eresume_code_cmp_patch2\n"
		".globl enter_eresume_code_enter_page_patch\n"

		//NOTE WELL - this routine uses all the defines from
		//test_code (async exit flow)

		"enter_eresume_code:\n"
		"enter_eresume_code_itp_loop:\n"
		"mov eax, 3;\n"
		"nop;\n"
		"nop;\n"
		"nop;\n"
		"nop;\n"
		"cmp eax, 4;\n"
		"jz enter_eresume_code_itp_loop;\n"
		#if 0
		"mov eax, [esp+vector];\n"
		"cmp eax, kenter_vector;\n"
		"jnz enter_eresume_code_step_1;\n"
		"loop_kenter:\n"
		"jmp loop_kenter;\n"
		#endif
		"enter_eresume_code_step_1:\n"
		//We are entering this code through an
		//interrupt gate then we dont need to disable interrupts

		//Save GPRS (RCX specifically) as passed in by EENTER
		//since RCX is used to switch views
		//KCZ ecx was pushed onto stack in VIDT_STUB as the second value after edx

		//View handle passed in via RDX, load into RCX

		"enter_eresume_code_cpuindex:\n"
		"mov eax, patch_str_core_id;\n"
		"push eax;\n" //save core id on stack

		//this stack frame is copied over to enter-page at this point:
		//VT: Note that for KENTER, SS and ESP will not be copied on the stack.
		//+------------+
		//| SS         | 56
		//+------------+
		//| ESP        | 52
		//+------------+
		//| EFLAGS     | 48
		//+------------+
		//| CS         | 44
		//+------------+
		//| EIP        | 40
		//+------------+
		//| EDI        | 36
		//+------------+
		//| ESI        | 32
		//+------------+
		//| EBX        | 28
		//+------------+
		//| EDX        | 24
		//+------------+
		//| ECX        | 20
		//+------------+
		//| EAX        | 16
		//+------------+
		//| VECTOR     | 12
		//+------------+
		//| ERRORFLAG  |  8
		//+------------+
		//| EIP (CALL) |  4
		//+------------+
		//| coreid     |  0
		//+------------+

		//STACKFIX Ravi - copy stack contents from untrusted (unknown) stack
		//to pre known (untrusted) enter page buffer
		//#### get enter-page for cpu
		"enter_eresume_code_enter_page_patch:\n"
		"mov eax, patch_str_enter_page;\n"
		//#### cache enter-page address in edi
		"mov edi, eax;\n" //edi == enter_page
		//#### COPY STACK contents in bulk (whole stack frame) to enter-page<rw>
		"pop eax;\n" //core id
		"mov [edi], eax;\n"
		"add edi, 4;\n" //Adding, so that subsequent code can use the offset macros
		"pop eax;\n" //eip
		"mov [edi], eax;\n"
		"pop eax;\n" //flags
		"mov [edi+flag],eax;\n"
		"pop eax;\n" //vector
		"mov esi, eax;\n" //esi == vector
		"mov [edi+vector],eax;\n"
		"pop eax;\n" //eax
		"mov [edi+start_gprs+gprs.eax], eax;\n"
		"pop eax;\n" //rcx
		"mov [edi+start_gprs+gprs.ecx], eax;\n"
		"pop eax;\n" //rdx
		"mov [edi+start_gprs+gprs.edx], eax;\n"
		"pop eax;\n" //rbx
		"mov [edi+start_gprs+gprs.ebx], eax;\n"
		"pop eax;\n" //rsi
		"mov [edi+start_gprs+gprs.esi], eax;\n"
		"pop eax;\n" //rdi
		"mov [edi+start_gprs+gprs.edi], eax;\n"
		"pop eax;\n" //rip
		"mov [edi+if_noskip_ec+if.eip], eax;\n"
		"pop eax;\n" //cs
		"mov [edi+if_noskip_ec+if.cs], eax;\n"
		"pop eax;\n" //rflags
		"mov [edi+if_noskip_ec+if.eflags], eax;\n"

		//"cmp esi, kenter_vector;\n" //Intra-privilege 'int' call doesn't push SS and ESP on the interrupt stack frame.
		"mov eax, [edi+if_noskip_ec+if.cs];\n"
		"and eax, ss_rpl_mask;\n"
		"cmp eax, ss_rpl_ring0;\n"
		"jz check_secs;\n"
		"pop eax;\n" //rsp
		"mov [edi+if_noskip_ec+if.esp], eax;\n"
		"pop eax;\n" //ss
		"mov [edi+if_noskip_ec+if.ss], eax;\n"
		"check_secs:\n"
		//#### adjust untrusted rsp to remove frame - already done above in pops
		//#### cache untrusted r0 stack in [edi+ur0sp] since after we switch tss will be
		//remapped and we will need to save this - should go into secs pcd OR TCS??
		"mov [edi+ur0sp], esp;\n"
		// my assumption is that eenter/eresume flow is called from myhandler
		// if that's the case, I alrady have esp

		//Save GPRS (RCX specifically) as passed in by EENTER since RCX is used to switch views

		//View handle passed in via RDX, load into RCX for EPTP switching
		"mov ecx, [edi+start_gprs+gprs.edx];\n"

		// edx is now available

		//Check that the Current View is zero (entry should be
		//invoked by untrusted code)
		"enter_eresume_code_secs_patch1:\n"
		"mov edx, patch_str_secs_ptr;\n"
		"mov eax, [edx+secs.scv];\n"
		"enter_eresume_code_cmp_patch1:\n"
		"cmp eax, patch_str_secs_scv_un;\n"
		"jz view_0_secs_check_ok;\n"
		"mov ecx, 0x1;\n"
		"mov edx, 0x1;\n"
		"jmp gp_vmcall;\n"

		"view_0_secs_check_ok:\n"
		//KCZ must be 0; otherwise, EENTER is illegal
		"mov ax, [edx+secs.eid];\n"
		"cmp ax, 0;\n"
		"jz view_0_check_ok;\n"

		"mov ecx, view_0_chk_fail;\n"
		"mov edx, view_0_chk_fail;\n"
		"jmp gp_vmcall;\n"

		"view_0_check_ok:\n"

		//VMM sets/resets the ept_enabled flag based on the
		//presence or absence of active views. If EPT is disabled.
		//then we should not try to do vmfunc.
		"mov eax, [edx+secs.ept_enabled];\n"
		"bt eax, 0;\n"
		"jnc cancel_enter_no_view_switch;\n"
		//"cmp esi, kenter_vector;\n"
		//Check if rpl is 0, which means intra priv interrupt call
		// that came from KTA
		"mov eax, [edi+if_noskip_ec+if.cs];\n"
		"and eax, ss_rpl_mask;\n"
		"cmp eax, ss_rpl_ring0;\n"
		"jz do_vmfunc;\n"
		// Raise NM# exception before
		// view switch, so that we never get "Device
		// not available" exception in trusted code.
		// fwait causes an NM# if CR0.TS flag is set.
		// The flag is set by the OS to delay the restoring
		// of FPU context on a context switch. Only when the
		// FPU instruction (like fwait, or wait) is execute,
		// the NM# exception in raised on which the OS resets
		// CR0.ts, thereby preventing further NM#
		"mov eax, cr0;\n"
		"bt eax, cr0.ts;\n"
		"jnc do_vmfunc;\n"
		"fwait;\n" // Will raise NM# exception
		//Note that for 32 bit, we don't switch the GS to kernel GS as the
		//kernel exception handler sets the kernel GS. This is unlike
		//64 bit where we need to use swapgs before raising an exception
		"do_vmfunc:\n"

		//****************debug only start***************
		/*
		//VIVEK: update secs_ptr->base with address passed in eax
		"mov eax, [esp+12];\n" //get vector
		"cmp eax, eresume_vector;\n"
		"je eresume_flow;\n"

		//do this only for enter flow (since for resume it should
		//already be set correctly)
		"mov eax, [esp+start_gprs+gprs.eax +4];\n" // eax in stack contains the SEC base
		"mov [edx+secs.base], eax; \n" // secs->base = SEC base passed through eenter

		"eresume_flow:\n"

		"mov ebx, edx;\n"
		"add ebx, secs.eid;\n"
		"mov ax, 0;\n" //expect view 0
		"mov cx, 1;\n" //switch eid to 1
		".byte 0xF0;\n"
		"cmpxchg [ebx], cx;\n"
		 */
		//****************debug only end***************

		//Perform vmfunc to transition to required EPT trusted view
		//KCZ ecx has the view number already
		"mov eax, vmfunc_view_sw_ctrl;\n"
		//"vmcall;\n" //VMCALL only for debug
		//VMFUNC opcode emitted below
		".byte 0x0f;\n"
		".byte 0x01;\n"
		".byte 0xd4;\n"

		//TODO: Check if the return value is coming in right register
		//RAHIL: Now, eax is properly updated
		"cmp eax, vmfunc_return_success;\n"
		"jne cancel_enter_eresume1;\n"
		"mov eax, esi;\n" //eax == vector
		//KCZ views are switched; should load cr3 but not doing that for now...
		//KCZ but before the switch, we have to store OS-cr3 for now before
		//KCZ we can save it in SECS
		//KCZ esi should be available
		"mov esi, cr3;\n" //OS-CR3 will be saved in TCS later.
		//"cmp eax, kenter_vector;\n" //No need for checking caller process if entry is for KTA
		"mov eax, [edi+if_noskip_ec+if.cs];\n"
		"and eax, ss_rpl_mask;\n"
		"cmp eax, ss_rpl_ring0;\n"
		"jz check_scv_trusted_view;\n"

		"mov eax, [edx+secs.os_cr3];\n"
		"cmp esi, eax;\n"
		"je check_scv_trusted_view;\n"
		"cancel_enter_eresume1:\n"
		"mov esi, eax;\n"
		"cancel_enter_eresume:\n"

		"mov eax, vmfunc_view_sw_ctrl;\n"
		"mov ecx, untrusted_view_id;\n"
		// Switch back to view0
		".byte 0x0f;\n"
		".byte 0x01;\n"
		".byte 0xd4;\n"

		"cancel_enter_no_view_switch:\n"

		//Recreate the untrusted stack again as it was emptied
		//at the beginning with an intention to switch to the
		//trusted view
		"mov eax, [edi+if_noskip_ec+if.ss];\n"
		"push eax;\n"
		"mov eax, [edi+if_noskip_ec+if.esp];\n"
		"push eax;\n"
		"mov eax, [edi+if_noskip_ec+if.eflags];\n"
		"push eax;\n"
		"mov eax, [edi+if_noskip_ec+if.cs];\n"
		"push eax;\n"
		"mov eax, [edi+if_noskip_ec+if.eip];\n"
		"push eax;\n"
		"mov eax, [edi+start_gprs+gprs.edi];\n"
		"push eax;\n"
		"mov eax, [edi+start_gprs+gprs.esi];\n"
		"push eax;\n"
		"mov eax, [edi+start_gprs+gprs.ebx];\n"
		"push eax;\n"
		"mov eax, [edi+start_gprs+gprs.edx];\n"
		"push eax;\n"
		"mov eax, [edi+start_gprs+gprs.ecx];\n"
		"push eax;\n"
		"mov eax, [edi+start_gprs+gprs.eax];\n"
		"push eax;\n"
		"mov eax, [edi+vector];\n"
		"push eax;\n"
		"mov eax, [edi+flag];\n"
		"push eax;\n"
		"mov eax, [edi];\n" //rip in VIDT_STUB
		"push eax;\n"
		"sub edi, 4;\n" //edi now points to [core id]
		"mov eax, [edi];\n"
		"push eax;\n" //core id

		//enter_enclave.S expects %edi as -1 for normal
		//exit, any other value of %edi is treated as
		//ocall return
		"mov edi, -1;\n" //Don't need edi anymore
		"mov [esp+start_gprs+gprs.edi+4], edi;\n"
		//esi expects the return status from ecall.
		//return nonzero value
		"mov esi, ecall_not_allowed;\n"
		"mov [esp+start_gprs+gprs.esi+4], esi;\n"

		"jmp out_enter_eresume;\n"

		"check_scv_trusted_view:\n"
		//Read SECS security cookie from SECS GVA (RDI)
		//Compare against immediate value on XO trampoline page
		"mov eax, [edx+secs.scv];\n"
		"enter_eresume_code_cmp_patch2:\n"
		"cmp eax, patch_str_secs_scv;\n"
		"jz view_x_secs_check_ok;\n"

		"mov ecx, secs_scv_chk_fail;\n"
		"mov edx, secs_scv_chk_fail;\n"
		"jmp gp_vmcall;\n"

		"view_x_secs_check_ok:\n"
		//Read view-handle expected from SECS
		//Verify view handle requested post view switch
		//and branch to fail if mis-match
		//(Fail vmcalls and injects #GP after restoring OS-CR3 and RCX)
		//KCZ ecx still has view id from the original edx
		"mov ax, [edx+secs.eid];\n"
		"cmp ax, cx;\n"
		"jz view_x_check_ok;\n"
		//VT_TEMP
		"mov edx, 0xa;\n"
		"mov esi, ecx;\n"
		"jmp gp_vmcall;\n"
		//Handle cases where some other thread destroys
		//the view while this thread is trying to enter
		//the view. Rather than causing panic in the
		//VMM, we exit out to the guest with reason of
		//ecall not allowed
		"jmp cancel_enter_eresume;\n"

		"view_x_check_ok:\n"
		//Verify SECS state to ensure View initialized
		//KCZ secs->attributes->init flag?
		//KCZ bit 0 of secs->attributes has to be set
		"mov eax, [edx+secs.attrib];\n"
		"bt eax, 0;\n"
		"jc secs_inited_check_ok;\n"
		//VT_TEMP
		"mov edx, 0xb;\n"
		"mov esi, eax;\n"
		"jmp gp_vmcall;\n"

		"jmp cancel_enter_eresume;\n"

		"secs_inited_check_ok:\n"
		//Cache OS-CR3 in GPR
		//KCZ shouldn't this have been done above?
		//KCZ in any case, OS-cr3 is in esi
		//load ACR3 from SECS
		//KCZ again, shouldn't this be done above before
		//KCZ we access secs_la?
		#ifdef ACTIVATE_ACR3
		"mov eax, [edx+secs.acr3];\n"
		"mov cr3, eax;\n"
		#endif

		//STACKFIX Ravi
		//cpu tss points to our tss which refs our r0 stack
		//and we can now access our r0 stack, but we have to set it up and pivot first
		//###save untrusted r0 rsp from r14 into TCS -> done later when TCS ready
		//###copy bulk info from enter-page<ro> r15 to new rsp from secs pcd
		//###pivot stack to trusted r0 stack
		"xor ecx, ecx;\n"
		"mov ebx, edx;\n" //secs ptr
		"add ebx, secs.pcd;\n"
		"mov ecx, [edi-4];\n" //core id cached at [edi-4]
		"shl ecx, shift_size_secs_pcd;\n"
		"add ecx, ebx;\n" //ptr to core-specific pcd
		"mov esp, [ecx+secs.pcd.r0sp];\n"
		"mov ebx, [edi+start_gprs+gprs.ebx];\n" // ebx = tcs_t *
		"mov [ebx+tcs.tr0sp], esp;\n" //KTSL would switch to this stack on entry.
		"mov eax, [edi+if_noskip_ec+if.cs];\n"
		"and eax, ss_rpl_mask;\n"
		"cmp eax, ss_rpl_ring0;\n"
		//  "mov eax, [edi+vector];\n"
		//  "cmp eax, kenter_vector;\n"
		"jz copy_rem_sf;\n"
		"mov eax, [edi+if_noskip_ec+if.ss];\n"
		"push eax;\n"
		"mov eax, [edi+if_noskip_ec+if.esp];\n"
		"push eax;\n"
		"copy_rem_sf:\n"
		"mov eax, [edi+if_noskip_ec+if.eflags];\n"
		"push eax;\n"
		"mov eax, [edi+if_noskip_ec+if.cs];\n"
		"push eax;\n"
		"mov eax, [edi+if_noskip_ec+if.eip];\n"
		"push eax;\n"
		"mov eax, [edi+start_gprs+gprs.edi];\n"
		"push eax;\n"
		"mov eax, [edi+start_gprs+gprs.esi];\n"
		"push eax;\n"
		"mov eax, [edi+start_gprs+gprs.ebx];\n"
		"push eax;\n"
		"mov eax, [edi+start_gprs+gprs.edx];\n"
		"push eax;\n"
		"mov eax, [edi+start_gprs+gprs.ecx];\n"
		"push eax;\n"
		"mov eax, [edi+start_gprs+gprs.eax];\n"
		"push eax;\n"
		"mov eax, [edi+vector];\n"
		"push eax;\n"
		"mov eax, [edi+flag];\n"
		"push eax;\n"
		"mov eax, [edi];\n" //caller eip
		"push eax;\n"
		"sub edi, 4;\n" //edi now points to base of enter_page
		"mov eax, [edi];\n" //core_id
		"push eax;\n"

		"xor ebx, ebx;\n"
		//Verify RBX is page-aligned (for TCS)
		//KCZ if we are supposed to save OS-cr3 in TCS
		//KCZ shouldn't this check be done earlier?
		//KCZ !(rbx & 4095) is OK
		"mov ebx, [esp+28];\n"
		"mov eax, ebx;\n"
		"and eax, 4095;\n"
		"cmp eax, 0;\n"
		"jz tcs_page_aligned_check_ok;\n"

		"mov ecx, tcs_pg_align_chk_fail;\n"
		"mov edx, tcs_pg_align_chk_fail;\n"
		"jmp gp_vmcall;\n"

		"tcs_page_aligned_check_ok:\n"

		//STACKFIX Save untrusted stack we entered with
		"mov eax, [edi+4+ur0sp];\n" //ur0sp +4 to accomodate for core-id push. When ur0sp was saved, core-id on the stack was not factored in
		"mov [ebx+tcs.ur0sp], eax;\n"
		"mov [ebx+tcs.os_cr3], esi;\n"
		//TODO: Remove the below line once we enable code
		//from jz modify_eflags included code.
		//RAHIL: Done
		"mov eax, [esp+4+if_noskip_ec+if.cs];\n"
		"and eax, ss_rpl_mask;\n"
		"cmp eax, ss_rpl_ring0;\n"
		//"mov eax,[edi+vector+4];\n"
		//"cmp eax, kenter_vector;\n"
		"jz push_ssa_os_cr3_kta;\n"

		// save OS-CR3 until we can write to TCS
		"push esi;\n"

		//Make sure SSA contains at least one frame

		// cssa
		"mov eax, [ebx+tcs.cssa];\n"
		// if this is eresume, jmp to a different check
		"mov esi, [esp+12+4];\n" //get vector
		"cmp esi, eresume_vector;\n"
		"jz check_ssa_for_eresume;\n"

		// compare cssa to nssa
		"mov esi, [ebx+tcs.nssa];\n"
		"cmp eax, esi;\n"
		// cssa must be < nssa
		"jb ssa_frame_avail_check_ok;\n"

		"mov ecx, ssa_frame_avl_chk1_fail;\n"
		"mov edx, ssa_frame_avl_chk1_fail;\n"
		"jmp gp_vmcall;\n"

		"check_ssa_for_eresume:\n"
		// there must be at least one active frame
		"cmp eax, 0;\n"
		"jnz ssa_frame_avail_check_ok;\n"

		"mov ecx, ssa_frame_avl_chk2_fail;\n"
		"mov edx, ssa_frame_avl_chk2_fail;\n"
		"jmp gp_vmcall;\n"

		"ssa_frame_avail_check_ok:\n"

		// calculate ssa
		//uint64_t ssa_start = tcs->arch.ossa + secs->baseaddr
		//+ SSA_FRAME_SIZE(secs) * tcs->arch.cssa;

		// assume ssa frame size is 4096
		// eax has cssa already
		"push edx;\n" //since mul clobbers eax, edx
		"mov ecx, 4096;\n"
		"mul ecx;\n" //RAVI modified to use ecx - confirm ok
		// KCZ yes, it is OK; I forgot mul must use r/m32
		"pop edx;\n" //since mul clobbers eax, edx - restore secs

		// add ossa
		"add eax, [ebx+tcs.ossa];\n"

		// add base address from SECS
		"add eax, [edx+secs.base];\n"

		// if this is eresume, subtract one frame
		"mov ecx, [esp+12+4];\n" //get vector
		"cmp ecx, eresume_vector;\n"
		"jnz ssa_address_done;\n"
		// for now it is hardcoded to one page
		"sub eax, 4096;\n"
		"ssa_address_done:\n"
		// push on the stack, it will be needed later
		"push eax;\n"
		"jmp perform_segment_checks;\n"
		"push_ssa_os_cr3_kta:\n"
		"push esi;\n"
		"mov eax, 0;\n"
		"push eax;\n" //For kta, ssa is 0. It is not used
		// the stack looks like this
		//+------------+
		//| os-cr3     |  4
		//+------------+
		//| ssa        |  0
		//+------------+

		"perform_segment_checks:\n"
		//Perform segment table validation
		//Verify that CS, SS, DS, ES.base is 0
		// KCZ GDT and LDT bases are cached in SECS
		// KCZ because desc table exiting is on
		"mov eax, [esp+8];\n" //core id was at 0 now at 8
		"shl eax, shift_size_secs_pcd;\n"
		"add eax, edx;\n"
		"add eax, secs.pcd;\n"

		"mov esi, [eax+secs.pcd.gdtr];\n"
		"mov edi, [eax+secs.pcd.ldtr];\n"

		// GP_IF (cs.base)
		"xor eax, eax;\n"
		"mov ax, cs;\n"
		CHECK_SEGMENT_BASE("cs")

		//    GP_IF (aep > cs.limit)         ;
		// now edx has lo; ecx has hi
		// limit is 16 lsbs from edx and bits 16-19 from ecx
		// this is cs so no expanding down
		"and edx, 0xFFFF;\n"

		//check granularity of segment
		"bt ecx, 23;\n"
		"jc seg_g_4k_inc;\n"

		//byte granularity
		"and ecx, 0xF0000;\n"
		"or ecx, edx;\n"

		"jmp perf_aep_check;\n"

		"seg_g_4k_inc:\n"
		"and ecx, 0xF0000;\n"
		"or ecx, edx;\n"
		"shl ecx, shift_size_4k;\n"

		"perf_aep_check:\n"

		"mov eax, [esp+4+if_noskip_ec+if.cs+8];\n"
		"and eax, ss_rpl_mask;\n"
		"cmp eax, ss_rpl_ring0;\n"
		"jz kta_continue_seg_check;\n"
		"mov edx, [ebx+tcs.aep];\n"
		"cmp ecx, edx;\n"
		"jl aep_check_ok;\n"

		"mov ecx, aep_chk_fail;\n"
		"mov edx, aep_chk_fail;\n"
		"jmp gp_vmcall;\n"

		"aep_check_ok:\n"

		// if eenter
		//    target = tcs->arch.oentry + secs->baseaddr;
		// if eresume
		//    target = gpr->rip
		//    GP_IF (target > cs.limit);

		"mov eax, [esp+12+8];\n" //vector was at 12 now at 20
		"cmp eax, eresume_vector;\n"
		"jz eresume_target;\n"

		// oentry
		"mov eax, [ebx+tcs.oentry];\n"
		// add base address from SECS
		"enter_eresume_code_secs_patch2:\n"
		"mov edx, patch_str_secs_ptr;\n"
		"add eax, [edx+secs.base];\n"
		"jmp target_check;\n"

		"eresume_target:\n"
		// ssa is on the stack
		"mov edx, [esp];\n"
		//but gprs are at the end
		"add edx, 4096;\n"
		"sub edx, ssa_gpr_size;\n"
		"mov eax, [edx+ssa.ip];\n"

		"target_check:\n"
		"cmp ecx, eax;\n"
		"jl target_check_ok;\n"

		"mov ecx, target_chk_fail;\n"
		"mov edx, target_chk_fail;\n"
		"jmp gp_vmcall;\n"

		"target_check_ok:\n"

		// KCZ temp fix
		// moved ds checks after ss checks
		// KCZ temp fix

		// the problem is that USABLE macro in PAS uses a null bit in AR
		// I don't know if such a bit exists here bt if base is not 0
		// but segement is somehow marked unusable, we will inject #GP
		//    if (USABLE(es)) GP_IF (es.base);
		//    if (USABLE(ss)) GP_IF (ss.base || !ss.ar.db);
		"kta_continue_seg_check:\n"

		"xor eax, eax;\n"
		"mov ax, es;\n"
		CHECK_SEGMENT_BASE("es")

		"xor eax, eax;\n"
		"mov ax, ss;\n"
		CHECK_SEGMENT_BASE("ss")

		// ecx has ss.hi, check db bit
		"bt ecx, 22;\n"
		"jc ss_db_check_ok;\n"

		"mov ecx, ss_db_chk_fail;\n"
		"mov edx, ss_db_chk_fail;\n"
		"jmp gp_vmcall;\n"

		"ss_db_check_ok:\n"

		// KCZ temp fix
		//    GP_IF (ds.base)
		"xor eax, eax;\n"
		"mov ax, ds;\n"
		CHECK_SEGMENT_BASE("ds")
		//TODO: RAHIL put a check for kenter
		//and by pass all the checks till end comment
		//RAHIL: Done
		"push ecx;\n"
		"mov eax, [esp+4+if_noskip_ec+if.cs+12];\n"
		"and eax, ss_rpl_mask;\n"
		"cmp eax, ss_rpl_ring0;\n"
		"jz check_tcs_state;\n"

		// this check is in the PAS; ucode tests this by testing ebx against ds.limit
		// seems easier to do it the PAS way
		"bt ecx, 10;\n"
		"jnc ds_expand_check_ok;\n"

		"mov ecx, ds_expand_chk_fail;\n"
		"mov edx, ds_expand_chk_fail;\n"
		"jmp gp_vmcall;\n"

		"ds_expand_check_ok:\n"
		// KCZ temp fix

		// check SSA and FS/GS base	alignment
		"mov eax, [ebx+tcs.ossa];\n"
		"and eax, 4095;\n"
		"cmp eax, 0;\n"
		"jz ossa_page_aligned_check_ok;\n"

		"mov ecx, 0xd;\n"
		"mov edx, 0xd;\n"
		"jmp gp_vmcall;\n"

		"ossa_page_aligned_check_ok:\n"
		// tcs.ofs_base
		"mov eax, [ebx+48];\n"
		"and eax, 4095;\n"
		"cmp eax, 0;\n"
		"jz ofsbase_page_aligned_check_ok;\n"

		"mov ecx, ofsbase_page_align_chk_fail;\n"
		"mov edx, ofsbase_page_align_chk_fail;\n"
		"jmp gp_vmcall;\n"

		"ofsbase_page_aligned_check_ok:\n"
		// tcs.ogs_base
		"mov eax, [ebx+56];\n"
		"and eax, 4095;\n"
		"cmp eax, 0;\n"
		"jz ogsbase_page_aligned_check_ok;\n"

		"mov ecx, ogsbase_page_align_chk_fail;\n"
		"mov edx, ogsbase_page_align_chk_fail;\n"
		"jmp gp_vmcall;\n"

		"ogsbase_page_aligned_check_ok:\n"

		// check that proposed FS/GS segments fall within DS
		// edx has ds.lo; ecx has ds.hi
		"and edx, 0xFFFF;\n"

		//check granularity of segment
		"bt ecx, seg_granularity;\n"
		"jc ds_g_4k_inc;\n"

		//byte granularity
		"and ecx, 0xF0000;\n"
		"or ecx, edx;\n"

		"jmp perf_fsgs_check;\n"

		"ds_g_4k_inc:\n"
		"and ecx, 0xF0000;\n"
		"or ecx, edx;\n"
		"shl ecx, shift_size_4k;\n"

		"perf_fsgs_check:\n"
		// ecx has granularity adjusted ds.limit now
		"enter_eresume_code_secs_patch3:\n"
		"mov edx, patch_str_secs_ptr;\n"
		"mov eax, [edx+secs.base];\n"
		"add eax, [ebx+48];\n"
		"add eax, [ebx+64];\n"
		// eax has enclave_base + ofs_base + ofs_limit
		// first, check for overflow (wrap-around)
		// if overflow, ds.limit must be more than 4GB, which is not possible
		// otherwise eax must be less than or equal to ds.limit
		"jnc no_fs_wrap_around_check_ok;\n"

		"mov ecx, no_fs_wrap_around_chk_fail;\n"
		"mov edx, no_fs_wrap_around_chk_fail;\n"
		"jmp gp_vmcall;\n"

		"no_fs_wrap_around_check_ok:\n"

		"cmp ecx, eax;\n"
		"jl fs_within_ds_check_ok;\n"

		"mov ecx, fs_within_ds_chk_fail;\n"
		"mov edx, fs_within_ds_chk_fail;\n"
		"jmp gp_vmcall;\n"

		"fs_within_ds_check_ok:\n"
		// now check gs
		"mov eax, [edx+secs.base];\n"
		"add eax, [ebx+56];\n"
		"add eax, [ebx+68];\n"
		"jnc no_gs_wrap_around_check_ok;\n"

		"mov ecx, no_gs_wrap_around_chk_fail;\n"
		"mov edx, no_gs_wrap_around_chk_fail;\n"
		"jmp gp_vmcall;\n"

		"no_gs_wrap_around_check_ok:\n"

		"cmp ecx, eax;\n"
		"jl gs_within_ds_check_ok;\n"

		"mov ecx, gs_within_ds_chk_fail;\n"
		"mov edx, gs_within_ds_chk_fail;\n"
		"jmp gp_vmcall;\n"

		"gs_within_ds_check_ok:\n"

		// check if TCS.flags.reserved is all 0s
		"mov eax, [ebx+tcs.flags];\n"
		"and eax, 0xFFFFFFFE;\n"
		"jz tcs_reserved_0_3_check_ok;\n"

		"mov ecx, tcs_reserved_chk_fail;\n"
		"mov edx, tcs_reserved_chk_fail;\n"
		"jmp gp_vmcall;\n"

		"tcs_reserved_0_3_check_ok:\n"

		"mov eax, [ebx+tcs.flags+4];\n"
		"and eax, 0xFFFFFFFF;\n"
		"jz tcs_reserved_4_7_check_ok;\n"

		"mov ecx, tcs_lock_acquire_fail;\n"
		"mov edx, tcs_lock_acquire_fail;\n"
		"jmp gp_vmcall;\n"

		"tcs_reserved_4_7_check_ok:\n"

		// Transition to next state: INACTIVE -> ACTIVE
		// Make sure we started in the INACTIVE state and are the only thread using
		// the TCS. Otherwise, the instruction fails w/ #GP
		// TCS must not be active
		//TODO: enable below code ion tcs state checks as well
		//RAHIL bypass above code
		//RAHI: DONE
		"check_tcs_state:\n"
		"mov eax, 0;\n"
		"mov ecx, 1;\n" //RAVI added
		// add lock
		".byte 0xF0;\n"
		// set to active
		//cmpxchg [ebx], 1;\n"
		"cmpxchg [ebx], ecx;\n" //RAVI modified
		"mov [ebx], ecx;\n" //RAVI modified
		"je tcs_lock_acquired_ok;\n"

		//VMCALL here to assert
		"mov esi, [ebx];\n"
		"mov edx, eax;\n"
		"mov ecx, vmcall_assert;\n"
		"mov eax, nr;\n"
		"mov ebx, service_type;\n"
		"vmcall;\n"

		"tcs_lock_acquired_ok:\n"

		"pop ecx;\n"
		"mov eax, [esp+4+if_noskip_ec+if.cs+8];\n"
		"and eax, ss_rpl_mask;\n"
		"cmp eax, ss_rpl_ring0;\n"
		"jz pop_dummy_ssa_kta;\n"
		// KCZ temp fix
		// KCZ temp fix
		////////// EENTER/ERESUME can't fail beyond this point /////////////////

		// we can now write to TCS
		// eax is available

		// tcs->ssa_featsave_ppa = ssa;
		"pop eax;\n"
		// save ssa in tcs
		"mov [ebx+tcs.ssa], eax;\n"

		// take are care of OS-CR3 and AEP
		//  tcs->arch.aep = aep; it's in ecx on the stack
		"mov eax, [esp+20+4];\n" //ecx at 20 but we still have cr3 on stack
		"mov [ebx+tcs.aep], eax;\n"
		"jmp continue_fs_gs_checks;\n"
		"pop_dummy_ssa_kta:\n"
		"pop eax;\n"//KTA: Remove dummy ssa from stack
		"continue_fs_gs_checks:\n"
		// OS-CR3
		"pop eax;\n"
		"mov [ebx+tcs.os_cr3], eax;\n"

		"mov eax, [esp+4+if_noskip_ec+if.cs];\n"
		"and eax, ss_rpl_mask;\n"
		"cmp eax, ss_rpl_ring0;\n"
		"jz modify_eflags;\n"
		// ecx has ds.hi
		// but we need to reuse it so save on stack
		// (could have pushed earlier instead of making copies)
		"push ecx;\n"

		// KCZ this comment is wrong; FS/GS are saved in TCS
		//Do the Guest FS/GS Swap from TCS  Save OS FS/GS to SSA
		"xor eax, eax;\n"
		"mov ax, fs;\n"
		// save in tcs
		"mov [ebx+tcs.save_fs_selector], ax;\n"
		"mov edx, eax;\n"
		// since we have to do *8 later
		// instead of shifting right by 3 and mult by 8
		// just do and 0xFFF8
		//shr eax, 3
		"and eax, 0xFFF8;\n"
		"and edx, 4;\n"
		"cmp edx, 0;\n"
		"jne fs_use_ldt;\n"
		"add eax, esi;\n"
		// address of fs descriptor
		"push eax;\n"
		"mov ecx, [eax];\n"
		"mov eax, [eax+4];\n"
		"jmp fs_done;\n"
		"fs_use_ldt:\n"
		"add eax, edi;\n"
		// address of fs descriptor
		"push eax;\n"
		"mov ecx, [eax];\n"
		"mov eax, [eax+4];\n"
		"fs_done:\n"
		// save in tcs
		"mov [ebx+tcs.save_fs_desc_low], ecx;\n"
		"mov [ebx+tcs.save_fs_desc_high], eax;\n"

		"xor eax, eax;\n"
		"mov ax, gs;\n"
		"mov [ebx+tcs.save_gs_selector], ax;\n"
		"mov edx, eax;\n"
		//shr eax, 3
		"and eax, 0xFFF8;\n"
		"and edx, 4;\n"
		"cmp edx, 0;\n"
		"jne gs_use_ldt;\n"
		"add eax, esi;\n"
		// address of gs descriptor
		"push eax;\n"
		"mov ecx, [eax];\n"
		"mov eax, [eax+4];\n"
		"jmp gs_done;\n"
		"gs_use_ldt:\n"
		"add eax, edi;\n"
		// address of gs descriptor
		"push eax;\n"
		"mov ecx, [eax];\n"
		"mov eax, [eax+4];\n"
		"gs_done:\n"
		"mov [ebx+tcs.save_gs_desc_low], ecx;\n"
		"mov [ebx+tcs.save_gs_desc_high], eax;\n"

		// do the swap
		// stack looks like this now
		//+------------+
		//| ds.hi      |  8
		//+------------+
		//| &fs        |  4
		//+------------+
		//| &gs        |  0
		//+------------+

		// we start with ds descriptor
		"mov ecx, [esp+8];\n"

		// type &= 0x3
		// clear bits 10 and 11 in hi
		"and ecx, 0xFFFFF3FF;\n"
		// type |= 0x1
		// set bit 8
		// s = 1;
		// set bit 12
		// p = 1;
		// set bit 15
		// db = 1;
		// set bit 22
		// g = 1;
		// set bit 23
		"or ecx, 0xC09100;\n"

		// at this point FS/GS are the same
		// so copy back to ds.hi
		"mov [esp+8], ecx;\n"

		// write limit_15_00 and base_15_00
		"enter_eresume_code_secs_patch4:\n"
		"mov edx, patch_str_secs_ptr;\n"
		"mov eax, [edx+secs.base];\n"
		// ofs_base
		"add eax, [ebx+48];\n"
		// eax has the whole fs base; push for later
		"push eax;\n"
		// stack looks like this now
		//+------------+
		//| ds.hi      | 12
		//+------------+
		//| &fs        |  8
		//+------------+
		//| &gs        |  4
		//+------------+
		//| fs.b       |  0
		//+------------+

		// now build fs.lo
		// base_15_0 goes into msbs
		"shl eax, 16;\n"
		// limit_15_0 goes into lsbs
		// ofs_limit
		"mov ax, [ebx+64];\n"

		// eax has fs.lo now so write back
		// to the fs descriptor address on the stack
		"mov ecx, [esp+8];\n"
		"mov [ecx], eax;\n"

		// now gs.lo
		"mov eax, [edx+secs.base];\n"
		// ogs_base
		"add eax, [ebx+56];\n"
		// eax has the whole gs base; push for later
		"push eax;\n"
		// stack looks like this now
		//+------------+
		//| ds.hi      | 16
		//+------------+
		//| &fs        | 12
		//+------------+
		//| &gs        |  8
		//+------------+
		//| fs.b       |  4
		//+------------+
		//| gs.b       |  0
		//+------------+

		"shl eax, 16;\n"
		// ogs_limit
		"mov ax, [ebx+68];\n"
		"mov ecx, [esp+8];\n"
		"mov [ecx], eax;\n"

		// done with lo parts

		// read ds.hi
		"mov eax, [esp+16];\n"
		// we only need the bits in the middle
		"and eax, 0x00FFFF00;\n"
		// read fs.b
		"mov ecx, [esp+4];\n"
		// we only need 16 msbs
		"shr ecx, 16;\n"
		// copy 8 lsbs
		"or al, cl;\n"
		// now we need 8 msbs to be at the very top
		"shl ecx, 16;\n" //RAVI changed typo to shl
		// but don't need anything else
		"and ecx, 0xFF000000;\n"
		"or eax, ecx;\n"
		// we have the whole fs.hi now
		"mov ecx, [esp+12];\n"
		// so write back to the correct place
		"mov [ecx+4], eax;\n"

		// same thing for gs
		// read ds.hi
		"mov eax, [esp+16];\n"
		// we only need the bits in the middle
		"and eax, 0x00FFFF00;\n"
		// read gs.b
		"mov ecx, [esp];\n"
		// we only need 16 msbs
		"shr ecx, 16;\n"
		// copy 8 lsbs
		"or al, cl;\n"
		// now we need 8 msbs to be at the very top
		"shl ecx, 16;\n" //RAVI modified typo srl to shl - confirm
		// KCZ yes, shl is correct
		// but don't need anything else
		"and ecx, 0xFF000000;\n"
		"or eax, ecx;\n"
		// we have the whole fs.hi now
		"mov ecx, [esp+8];\n"
		// so write back to the correct place
		"mov [ecx+4], eax;\n"

		// finished swapping FS/GS
		// pop the stack
		"add esp, 20;\n"

		// shouldn't we force reload of FS/GS here?
		//RAVI - yes we should
		"mov fs, [ebx+tcs.save_fs_selector];\n"
		"mov gs, [ebx+tcs.save_gs_selector];\n"

		// if fs selector is 0 use gs selector
		// NOTE this only works if proposed fsbase == gsbase
		// and proposed fslim == gslim
		"xor eax, eax;\n"
		"mov ax, fs;\n"
		"cmp ax, 0;\n"
		"jne continue_enter_eresume;\n"
		"mov fs, [ebx+tcs.save_gs_selector];\n"
		"continue_enter_eresume:\n"

		// get stored ssa from tcs
		"mov eax, [ebx+tcs.ssa];\n"
		//gpr area is at the end of ssa page
		//  gpr = ssa + SE_PAGE_SIZE - sizeof(gpr_t);
		"add eax, 4096;\n"
		"sub eax, ssa_gpr_size;\n"

		"modify_eflags:\n"
		// eflags from the stack
		"mov edx, [esp+48];\n"
		//  tcs->rflags = rflags.raw;
		"mov [ebx+tcs.eflags], edx;\n"

		// if this is eresume, restore values stored by AEX
		"mov ecx, [esp+12];\n"
		"cmp ecx, eresume_vector;\n"
		"jz eresume_restore_state;\n"
		"mov ecx, [esp+if_noskip_ec+if.cs];\n"
		"and ecx, ss_rpl_mask;\n"
		"cmp ecx, ss_rpl_ring0;\n"
		//"cmp ecx, kenter_vector;\n"
		"jnz spu_bpu_save;\n"
		//Clear IF from EFLAGS
		"and edx, 0xFFFFFDFF;\n"
		"jmp clear_tf_eflags;\n"

		//Save state for possible asynchronous exits
		//Save the outside RSP and RBP so they can be restored
		//on next asynch or synch exit
		//Setup stack to be switched to trusted view stack by
		//setting RSP on interrupt frame

		"spu_bpu_save:\n"
		// save u_rsp and u_rbp
		"mov ecx, [esp+52];\n"
		"mov [eax+ssa.sp_u], ecx;\n"
		"mov [eax+ssa.bp_u], ebp;\n"

		// save EFLAGS.TF - needed only for OPTOUT enclave

		"clear_tf_eflags:\n"
		// clear TF
		"and edx, 0xFFFFFEFF;\n"

		"jmp prepare_for_enter;\n"

		"eresume_restore_state:\n"

		"mov eax, [ebx+tcs.ssa];\n"
		"fxrstor [eax];\n"

		"add eax, 4096;\n"
		"sub eax, ssa_gpr_size;\n"

		// eax
		"mov ecx, [eax+ssa.ax];\n"
		"mov [esp+16], ecx;\n"
		// ecx
		"mov ecx, [eax+ssa.cx];\n"
		"mov [esp+20], ecx;\n"
		// edx
		"mov ecx, [eax+ssa.dx];\n"
		"mov [esp+24], ecx;\n"
		// ebx
		"mov ecx, [eax+ssa.bx];\n"
		"mov [esp+28], ecx;\n"
		// esi
		"mov ecx, [eax+ssa.si];\n"
		"mov [esp+32], ecx;\n"
		// edi
		"mov ecx, [eax+ssa.di];\n"
		"mov [esp+36], ecx;\n"
		// ebp
		"mov ecx, [eax+ssa.bp];\n"
		"mov ebp, ecx;\n"
		// esp
		"mov ecx, [eax+ssa.sp];\n"
		"mov [esp+52], ecx;\n"

		// eflags [start]
		"mov edx, [eax+ssa.flags];\n"
		"mov ecx, [esp+48];\n"
		// restore tf
		"and ecx, 0x100;\n"
		"or edx, ecx;\n"
		"mov [esp+48], edx;\n"
		// eflags [end]

		//if resume flow should resume to ssa.ip
		"mov esi, [eax+ssa.ip];\n"

		// this is still eresume flow
		// must decrement cssa
		"mov eax, [ebx+24];\n"
		"dec eax;\n"
		"mov [ebx+24], eax;\n"

		"jmp eresume_restore_eip;\n"

		"prepare_for_enter:\n"
		// only eenter flow goes here.
		// eflags back on the stack
		//************debug only start************
		//"and edx, 0xFFFFFDFF;\n" //set IF=0 for test
		//************debug only end**************
		"mov [esp+48], edx;\n"

		//if enter then
		//Setup entrypoint RIP to TCS.OENTRY entrypoint by setting
		//RIP on interrupt frame

		// int saves correct address
		"mov eax, [esp+40];\n"
		// ecx
		"mov [esp+20], eax;\n"
		// cssa
		"mov eax, [ebx+tcs.cssa];\n"
		// eax
		"mov [esp+16], eax;\n"

		//IRET to continue execution at entrypoint point inside TV
		//if enter: entry point secs->base + tcs->oentry
		"enter_eresume_code_secs_patch5:\n"
		"mov edx, patch_str_secs_ptr;\n"
		"mov eax, [edx+secs.base];\n"
		"add eax, [ebx+tcs.oentry];\n"

		"mov [esp+40], eax;\n"
		"jmp enter_resume_last_step;\n"

		"eresume_restore_eip:\n"
		//if resume: entry point is from ssa.ip
		//which is stored in esi
		"mov [esp+40], esi;\n"

		"enter_resume_last_step:\n"
		// last step: write tcs address to secs[core#]
		"mov eax, [esp+0];\n"
		// size of per-core-data is shift_size_secs_pcd
		"shl eax, shift_size_secs_pcd;\n"
		"add eax, secs.pcd;\n"
		"add eax, secs.pcd.tcs;\n"
		"enter_eresume_code_secs_patch6:\n"
		"mov edx, patch_str_secs_ptr;\n" // Putting edx closer to its consumption
		"mov [edx+eax], ebx;\n"
		"out_enter_eresume:\n"
		"pop eax;\n"    //pop cpu id
		"mov eax, 0;\n" //return flag to indicate use IRET not RET
		"ret params_size;\n"
		//We return a flag in eax to my_handler so that it checks that
		//and either does a RET to the OS handler (like AEX) to address in edi OR
		//IRETs for our INT flows, in either case, my_handler leaves only the
		//required intr frame on the appropriate stack before issueing IRET or RET
		//IRET cleans up the intr frame, and RET does not (as intended)
		"gp_vmcall:\n"

		"mov eax, nr;\n"
		"mov ebx, service_type;\n"
		"mov ecx, vmcall_assert;\n"
		"vmcall;\n"

		".att_syntax\n"
);
void enter_eresume_code_end(void);
__asm__ (
		".globl enter_eresume_code_end\n"
		"enter_eresume_code_end:"
		"nop;\n"
	);
