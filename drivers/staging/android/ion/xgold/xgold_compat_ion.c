#include "../ion_priv.h"
#include <linux/uaccess.h>
#include <linux/compat.h>
#include <linux/xgold_ion.h>

int compat_get_xgold_ion_custom_data(
			struct compat_xgold_ion_get_params_data __user *data32,
			struct xgold_ion_get_params_data __user *data)
{
	compat_int_t handle;
	compat_size_t size;
	compat_long_t addr;
	int err;

	err = get_user(handle, &data32->handle);
	err |= put_user(handle, &data->handle);
	err = get_user(size, &data32->size);
	err |= put_user(size, &data->size);
	err = get_user(addr, &data32->addr);
	err |= put_user(addr, &data->addr);

	return err;

}

int compat_put_xgold_ion_custom_data(unsigned int arg, struct
		xgold_ion_get_params_data __user *data)
{
	struct compat_xgold_ion_get_params_data __user *data32;
	compat_int_t handle;
	compat_size_t size;
	compat_long_t addr;
	int err;

	data32 = compat_ptr(arg);

	err = get_user(handle, &data->handle);
	err |= put_user(handle, &data32->handle);
	err = get_user(size, &data->size);
	err |= put_user(size, &data32->size);
	err = get_user(addr, &data->addr);
	err |= put_user(addr, &data32->addr);

	return err;
}

struct xgold_ion_get_params_data __user
		*compat_xgold_ion_get_param(unsigned int arg)
{
	struct compat_xgold_ion_get_params_data __user *data32;
	struct xgold_ion_get_params_data __user *data;
	int ret = 0;

	data32 = compat_ptr(arg);
	data = compat_alloc_user_space(sizeof(*data));
	if (!data)
		return ERR_PTR(-EFAULT);

	ret = compat_get_xgold_ion_custom_data(data32, data);
	if (ret)
		return ERR_PTR(ret);

	return data;
}

