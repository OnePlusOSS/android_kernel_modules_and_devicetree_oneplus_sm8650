/*
 * Copyright (c) 2018-2021 The Linux Foundation. All rights reserved.
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all
 * copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

/**
 * DOC: Thin filesystem API abstractions
 */

#ifndef __QDF_FILE_H
#define __QDF_FILE_H

#include "qdf_status.h"

/**
 * qdf_file_read() - read the entire contents of a file
 * @path: the full path of the file to read
 * @out_buf: double pointer for referring to the file contents buffer
 *
 * This API allocates a new, null-terminated buffer containing the contents of
 * the file at @path. On success, @out_buf points to this new buffer, otherwise
 * @out_buf is set to NULL.
 *
 * Consumers must free the allocated buffer by calling qdf_file_buf_free().
 *
 * Return: QDF_STATUS
 */
QDF_STATUS qdf_file_read(const char *path, char **out_buf);

/**
 * qdf_file_read_bytes() - read the entire contents of a file and return the
 * size read along with the content
 * @path: the full path of the file to read
 * @out_buf: double pointer for referring to the file contents buffer
 * @out_buff_size: size of the contents read
 *
 * This API allocates a new, null-terminated buffer containing the contents of
 * the file at @path. On success, @out_buf points to this new buffer, otherwise
 * @out_buf is set to NULL.
 *
 * Consumers must free the allocated buffer by calling qdf_file_buf_free().
 *
 * Return: QDF_STATUS
 */
QDF_STATUS qdf_file_read_bytes(const char *path, char **out_buf,
			       unsigned int *out_buff_size);

/**
 * qdf_file_buf_free() - free a previously allocated file buffer
 * @file_buf: pointer to the file buffer to free
 *
 * This API is used in conjunction with qdf_file_read() and
 * qdf_file_read_bytes().
 *
 * Return: None
 */
void qdf_file_buf_free(char *file_buf);

#ifdef QCA_WIFI_MODULE_PARAMS_FROM_INI
/**
 * qdf_module_param_file_read() - read the entire contents of a file
 * @path: the full path of the file to read
 * @out_buf: double pointer for referring to the file contents buffer
 *
 * This API allocates a new buffer before qdf_mem_init() is being called.
 * Thus, this API helps to allocate memory which is being used before qdf
 * memory tracking framework is available. Buffer is null-terminated,
 * containing the contents of the file at @path. On success, @out_buf
 * points to this new buffer, otherwise @out_buf is set to NULL.
 *
 * Consumers must free the allocated buffer by calling
 * qdf_module_param_file_free().
 *
 * Return: QDF_STATUS
 */

QDF_STATUS qdf_module_param_file_read(const char *path, char **out_buf);

/**
 * qdf_module_param_file_free() - free a previously allocated file buffer
 * @file_buf: pointer to the file buffer to free. The buffer allocated in
 * qdf_module_param_file_read is not tracked by qdf framework.
 *
 * This API is used in conjunction with qdf_module_param_file_read().
 *
 * Return: None
 */
void qdf_module_param_file_free(char *file_buf);
#else
static inline
QDF_STATUS qdf_module_param_file_read(const char *path, char **out_buf)
{
	return QDF_STATUS_E_INVAL;
}

static inline
void qdf_module_param_file_free(char *file_buf)
{
}
#endif
#endif /* __QDF_FILE_H */

