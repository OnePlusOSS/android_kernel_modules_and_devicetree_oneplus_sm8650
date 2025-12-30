// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2025-2025 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "[DEBUG_AUTH]([%s][%d]): " fmt, __func__, __LINE__

#include <debug/oplus_debug_auth.h>

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <crypto/hash.h>
#include <crypto/skcipher.h>
#include <crypto/public_key.h>
#include <linux/crypto.h>
#include <linux/miscdevice.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
#include <linux/version.h>

#include <oplus_chg.h>
#include <oplus_chg_module.h>

#define OPLUS_DEBUG_CERT_MAGIC		0x20300606
#define OPLUS_DEBUG_CERT_USER_ID_LEN	16
#define OPLUS_DEBUG_CERT_SIGN_LEN	512
#define OPLUS_DEBUG_CERT_VALID_TIME_S	(3600 * 24 * 7) /* valid for 7 days */
#define OPLUS_DEBUG_TOKEN_SIZE		32
#define OPLUS_DEBUG_TOKEN_VALID_TIME_MS	(10 * 60 * 1000) /* 10 minutes */
#define OPLUS_DEBUG_DATA_MAGIC		0x20300608

struct oplus_debug_cert {
	uint32_t magic;
	uint8_t user_id[OPLUS_DEBUG_CERT_USER_ID_LEN];
	time64_t time;
	uint8_t sign[OPLUS_DEBUG_CERT_SIGN_LEN];
} __attribute__((packed));

struct oplus_debug_data_header {
	uint32_t magic;
	uint8_t token[OPLUS_DEBUG_TOKEN_SIZE];
	uint32_t size;
} __attribute__((packed));

struct oplus_debug_data {
	struct oplus_debug_data_header header;
	uint8_t data[];
} __attribute__((packed));

struct oplus_debug_auth {
	struct miscdevice auth_dev;
	struct oplus_debug_cert cert;
	bool cert_valid;
	uint8_t token[OPLUS_DEBUG_TOKEN_SIZE];
	uint8_t user_id[OPLUS_DEBUG_CERT_USER_ID_LEN];
	bool token_valid;
	struct delayed_work clear_token_work;
	struct mutex auth_lock;
};
struct oplus_debug_auth *g_oda;

static char public_auth_code[] = {
	0x30, 0x82, 0x02, 0x0A, 0x02, 0x82, 0x02, 0x01, 0x00, 0xE3, 0xBF, 0x47,
	0xB0, 0x0D, 0xEA, 0xC0, 0x45, 0xC0, 0xBF, 0x24, 0x31, 0x9E, 0xEC, 0x64,
	0xF8, 0x47, 0xF8, 0x58, 0x96, 0xCF, 0x5E, 0xDE, 0xC6, 0xF1, 0x90, 0x26,
	0xC9, 0x42, 0xF6, 0x21, 0x40, 0x70, 0x3D, 0x58, 0x26, 0x41, 0x6B, 0x7F,
	0x1E, 0x32, 0x46, 0x82, 0x1C, 0xCD, 0x77, 0x4B, 0xF1, 0xFB, 0xD7, 0x8F,
	0x70, 0x5D, 0x99, 0x1E, 0x0D, 0xD2, 0xB1, 0x9A, 0xEF, 0x9C, 0x2A, 0xC3,
	0x08, 0x0B, 0x79, 0x38, 0x49, 0x45, 0xAA, 0x92, 0xF1, 0x13, 0x08, 0x0F,
	0x6E, 0xBF, 0xCE, 0x26, 0x7F, 0x56, 0x75, 0x4F, 0x34, 0x11, 0x80, 0x39,
	0xA7, 0x2A, 0xCC, 0xA5, 0xF0, 0x35, 0x3A, 0x9C, 0x0F, 0x0B, 0xB2, 0xB5,
	0x2C, 0xC4, 0xE6, 0xC9, 0x1C, 0x79, 0x85, 0xFB, 0x33, 0x49, 0x9B, 0x6B,
	0x25, 0xC1, 0xD9, 0xE8, 0xA4, 0x21, 0xBF, 0xF0, 0x2D, 0xC2, 0x17, 0xD1,
	0xE1, 0xDC, 0x89, 0x2C, 0x22, 0x31, 0x32, 0x36, 0x26, 0xCD, 0x8A, 0xDD,
	0x08, 0x66, 0x82, 0x1B, 0xFB, 0x0D, 0x5C, 0x52, 0xEC, 0x6C, 0xE1, 0x14,
	0xC1, 0x66, 0xB9, 0x18, 0x3C, 0x49, 0x5D, 0xA6, 0xEE, 0x67, 0x2C, 0x30,
	0xB1, 0x64, 0xC1, 0x0B, 0x72, 0x0C, 0x2B, 0x8A, 0xBD, 0x59, 0x7B, 0xA5,
	0xE2, 0x3E, 0xB8, 0x89, 0x17, 0x00, 0x14, 0x40, 0x0D, 0x69, 0x79, 0x5A,
	0xF2, 0x44, 0x20, 0x56, 0xC8, 0x38, 0x06, 0xFA, 0xAB, 0x12, 0x2F, 0xEE,
	0x24, 0x83, 0xDA, 0x29, 0x45, 0x19, 0x91, 0x4A, 0x7A, 0x32, 0x22, 0x84,
	0x51, 0x67, 0x2F, 0x4A, 0xC6, 0x14, 0x7C, 0x42, 0x3E, 0x32, 0x16, 0x92,
	0x25, 0x4A, 0x1B, 0x1E, 0x2B, 0x0C, 0x74, 0x39, 0x96, 0x06, 0x9B, 0xB0,
	0x16, 0xFC, 0xCF, 0x40, 0xB4, 0x08, 0xE2, 0x95, 0x32, 0xF0, 0x48, 0x12,
	0x9B, 0xAA, 0x3B, 0xE6, 0xBB, 0x89, 0x0F, 0xA9, 0xAB, 0x95, 0x98, 0x7F,
	0x4D, 0x12, 0xE0, 0xC9, 0xAE, 0x5F, 0x5C, 0x72, 0x28, 0xF7, 0xBF, 0xC3,
	0xC0, 0x66, 0x35, 0xDF, 0x96, 0x54, 0x30, 0x03, 0xA5, 0xBC, 0xAF, 0xF9,
	0xD9, 0x7B, 0xE3, 0xB4, 0x4A, 0x2E, 0xF8, 0x0C, 0x6F, 0x58, 0x17, 0x84,
	0x9C, 0x30, 0x7F, 0xA4, 0x43, 0x3D, 0xA9, 0x2B, 0x55, 0xCA, 0x51, 0x77,
	0x5B, 0x32, 0xC0, 0xBA, 0x29, 0xCC, 0x2D, 0x8C, 0x6A, 0xA5, 0xD9, 0x57,
	0xCE, 0x55, 0x9E, 0x23, 0xE0, 0xC6, 0x8F, 0xF0, 0x3B, 0x85, 0xDF, 0xEC,
	0xE4, 0x27, 0x6C, 0xFB, 0x0F, 0xFF, 0xED, 0xBD, 0x83, 0x3D, 0x05, 0xF7,
	0xA2, 0xAE, 0xD4, 0xFD, 0x06, 0xA0, 0x71, 0x4C, 0x20, 0x0A, 0x64, 0xC9,
	0x40, 0x14, 0x6E, 0x51, 0xAD, 0x01, 0x28, 0x0C, 0x28, 0xC0, 0xF0, 0xE4,
	0x6A, 0xCC, 0x70, 0x66, 0x24, 0xA2, 0xEC, 0xFC, 0xA8, 0xCF, 0x40, 0xE5,
	0xE0, 0xC7, 0x8C, 0x74, 0x3B, 0x9E, 0xC0, 0x4E, 0x66, 0xDD, 0x52, 0x8B,
	0xF0, 0x53, 0x01, 0x3E, 0x60, 0x56, 0xE6, 0xA6, 0x1F, 0x71, 0x41, 0xFF,
	0x47, 0x0A, 0x5F, 0x2F, 0xA3, 0xE4, 0x93, 0xDA, 0x83, 0xB4, 0xF5, 0x98,
	0xE4, 0x56, 0xF7, 0x83, 0xBD, 0xFC, 0xA1, 0x6B, 0xDA, 0x1C, 0xF5, 0xAC,
	0x0B, 0xFA, 0x0A, 0xE6, 0x0B, 0xC1, 0x62, 0x24, 0x7B, 0xAB, 0x73, 0xB9,
	0xC1, 0xB0, 0xFF, 0xDB, 0xE2, 0x3A, 0x12, 0x61, 0x5F, 0xF7, 0xDF, 0x6E,
	0xFA, 0x56, 0x19, 0xC1, 0x9E, 0x42, 0xCC, 0x20, 0xD0, 0xFF, 0xA7, 0x54,
	0x30, 0x9F, 0xA6, 0x74, 0xEC, 0xB4, 0x3F, 0x19, 0x2B, 0xC5, 0x63, 0x8B,
	0xC8, 0x4B, 0xC7, 0x5E, 0xF4, 0x22, 0x88, 0x43, 0x7E, 0xD8, 0xCA, 0x33,
	0x85, 0x12, 0x9A, 0xEE, 0x76, 0x35, 0xF8, 0xB5, 0xF8, 0x19, 0x03, 0xA3,
	0xD4, 0x40, 0xD0, 0xAC, 0xD9, 0x40, 0x2A, 0xE4, 0xEF, 0x20, 0x3A, 0x66,
	0x09, 0x2C, 0xF9, 0x09, 0x99, 0x02, 0x03, 0x01, 0x00, 0x01
};

static char public_auth_code_params[] = {
	0x05, 0x00
};

static struct public_key oplus_pauth_code = {
	.key = public_auth_code,
	.keylen = ARRAY_SIZE(public_auth_code),
	.algo = OID_rsaEncryption,
	.params = public_auth_code_params,
	.paramlen = ARRAY_SIZE(public_auth_code_params),
	.key_is_private = false,
	.pkey_algo = "rsa",
};

static int oplus_debug_auth_dev_open(struct inode *inode, struct file *filp)
{
	struct oplus_debug_auth *oda =
		container_of(filp->private_data, struct oplus_debug_auth, auth_dev);

	filp->private_data = oda;
	return 0;
}

static int oplus_verify_signature(u8 *s, u32 s_size, u8 *digest, u8 digest_size)
{
	struct public_key_signature sig;

	sig.s = s;
	sig.s_size = s_size;
	sig.digest = digest;
	sig.digest_size = digest_size;
	sig.pkey_algo = "rsa";
	sig.hash_algo = "sha256";
	sig.encoding = "pkcs1";
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 10, 0) && LINUX_VERSION_CODE < KERNEL_VERSION(6, 5, 0))
	sig.data_size = 0;
	sig.data = NULL;
#endif

	return public_key_verify_signature(&oplus_pauth_code, &sig);
}

struct sdesc {
	struct shash_desc shash;
	char ctx[];
};

static struct sdesc *init_sdesc(struct crypto_shash *alg)
{
	struct sdesc *sdesc;
	int size;

	size = sizeof(struct shash_desc) + crypto_shash_descsize(alg);
	sdesc = kmalloc(size, GFP_KERNEL);
	if (!sdesc)
		return ERR_PTR(-ENOMEM);
	sdesc->shash.tfm = alg;
	return sdesc;
}

static int calc_hash(struct crypto_shash *alg, const unsigned char *data,
		     unsigned int datalen, unsigned char *digest)
{
	struct sdesc *sdesc;
	int ret;

	sdesc = init_sdesc(alg);
	if (IS_ERR(sdesc)) {
		chg_err("can't alloc sdesc\n");
		return PTR_ERR(sdesc);
	}

	ret = crypto_shash_digest(&sdesc->shash, data, datalen, digest);
	kfree(sdesc);
	return ret;
}

static int oplus_debug_auth_calc_digest(unsigned char *data, uint32_t size, unsigned char *digest)
{
	int rc;
	struct crypto_shash *alg;
	char *hash_alg_name = "sha256";

	alg = crypto_alloc_shash(hash_alg_name, 0, 0);
	if (IS_ERR(alg)) {
		chg_err("can't alloc alg %s\n", hash_alg_name);
		return PTR_ERR(alg);
	}
	rc = calc_hash(alg, data, size, digest);
	if (rc < 0) {
		chg_err("digest calculation failed, rc=%d\n", rc);
		crypto_free_shash(alg);
		return rc;
	}
	crypto_free_shash(alg);

	return 0;
}

static int oplus_debug_auth_verify_cert(struct oplus_debug_cert *cert)
{
	int rc;
	unsigned char digest[OPLUS_DEBUG_TOKEN_SIZE];

	rc = oplus_debug_auth_calc_digest((unsigned char *)cert,
		sizeof(struct oplus_debug_cert) - OPLUS_DEBUG_CERT_SIGN_LEN, digest);
	if (rc) {
		chg_err("cert digest calculation failed, rc=%d\n", rc);
		return rc;
	}

	rc = oplus_verify_signature(cert->sign, OPLUS_DEBUG_CERT_SIGN_LEN,
		digest, OPLUS_DEBUG_TOKEN_SIZE);
	if (rc < 0) {
		chg_err("cert signature verification failed, rc=%d\n", rc);
		return rc;
	}

	return 0;
}

static int oplus_debug_auth_set_cert(struct oplus_debug_auth *oda, void __user *buf)
{
	int rc;
	struct oplus_debug_cert *cert = &oda->cert;
	time64_t now;

	if (!oda || !buf) {
		chg_err("invalid oda or buf\n");
		return -EINVAL;
	}

	mutex_lock(&g_oda->auth_lock);
	rc = copy_from_user(cert, buf, sizeof(struct oplus_debug_cert));
	if (rc) {
		chg_err("failed copy to user space\n");
		goto err;
	}

	if (cert->magic != OPLUS_DEBUG_CERT_MAGIC) {
		chg_err("invalid cert magic\n");
		rc = -EINVAL;
		goto err;
	}

	now = ktime_get_real_seconds();
	if (now < cert->time || now > cert->time + OPLUS_DEBUG_CERT_VALID_TIME_S) {
		chg_err("cert expired: now=%llu, cert_time=%llu\n", now, cert->time);
		rc = -EINVAL;
		goto err;
	}

	rc = oplus_debug_auth_verify_cert(cert);
	if (rc) {
		chg_err("cert verification failed, rc=%d\n", rc);
		goto err;
	}

	oda->cert.user_id[OPLUS_DEBUG_CERT_USER_ID_LEN - 1] = 0;
	oda->cert_valid = true;
	memcpy(oda->user_id, oda->cert.user_id, OPLUS_DEBUG_CERT_USER_ID_LEN);
	mutex_unlock(&g_oda->auth_lock);
	chg_info("%s: set cert\n", oda->cert.user_id);
	return 0;

err:
	oda->cert_valid = false;
	memset(cert, 0, sizeof(struct oplus_debug_cert));
	mutex_unlock(&g_oda->auth_lock);
	return rc;
}

static int oplus_debug_auth_get_token(struct oplus_debug_auth *oda, void __user *buf)
{
	int rc;
	time64_t now;

	if (!oda || !buf) {
		chg_err("invalid oda or buf\n");
		return -EINVAL;
	}

	mutex_lock(&g_oda->auth_lock);
	if (!oda->cert_valid) {
		rc = -EACCES;
		chg_err("cert not valid\n");
		goto err;
	}

	now = ktime_get_real_seconds();
	if (now < oda->cert.time || now > oda->cert.time + OPLUS_DEBUG_CERT_VALID_TIME_S) {
		chg_err("cert expired: now=%llu, cert_time=%llu\n", now, oda->cert.time);
		rc = -EACCES;
		goto err;
	}

	rc = oplus_debug_auth_calc_digest((unsigned char *)&oda->cert, sizeof(struct oplus_debug_cert), oda->token);
	if (rc) {
		chg_err("token digest calculation failed, rc=%d\n", rc);
		goto err;
	}

	rc = copy_to_user(buf, oda->token, OPLUS_DEBUG_TOKEN_SIZE);
	if (rc) {
		chg_err("failed copy to user space\n");
		goto err;
	}

	chg_info("%s: get debug token\n", oda->cert.user_id);
	schedule_delayed_work(&oda->clear_token_work, msecs_to_jiffies(OPLUS_DEBUG_TOKEN_VALID_TIME_MS));
	oda->token_valid = true;
	mutex_unlock(&g_oda->auth_lock);
	return 0;
err:
	oda->token_valid = false;
	memset(oda->token, 0, OPLUS_DEBUG_TOKEN_SIZE);
	mutex_unlock(&g_oda->auth_lock);
	return rc;
}

static void oplus_debug_auth_clear_token_work(struct work_struct *work)
{
	struct oplus_debug_auth *oda =
		container_of(work, struct oplus_debug_auth, clear_token_work.work);

	mutex_lock(&oda->auth_lock);
	oda->token_valid = false;
	memset(oda->token, 0, OPLUS_DEBUG_TOKEN_SIZE);
	oda->cert_valid = false;
	memset(&oda->cert, 0, sizeof(struct oplus_debug_cert));
	mutex_unlock(&oda->auth_lock);
	chg_info("exit debug auth\n");
}

static int oplus_debug_auth_user_exit(struct oplus_debug_auth *oda, void __user *buf)
{
	int rc;
	uint8_t token[OPLUS_DEBUG_TOKEN_SIZE];

	if (!oda || !buf) {
		chg_err("invalid oda or buf\n");
		return -EINVAL;
	}

	if (!oda->token_valid)
		return 0;

	rc = copy_from_user(token, buf, OPLUS_DEBUG_TOKEN_SIZE);
	if (rc) {
		chg_err("failed copy from user space\n");
		return rc;
	}
	if (memcmp(token, oda->token, OPLUS_DEBUG_TOKEN_SIZE)) {
		chg_err("token mismatch\n");
		return -EINVAL;
	}

	cancel_delayed_work_sync(&oda->clear_token_work);
	mutex_lock(&g_oda->auth_lock);
	oda->token_valid = false;
	memset(oda->token, 0, OPLUS_DEBUG_TOKEN_SIZE);
	oda->cert_valid = false;
	memset(&oda->cert, 0, sizeof(struct oplus_debug_cert));
	mutex_unlock(&g_oda->auth_lock);
	chg_info("exit debug auth\n");

	return 0;
}

#define DEBUG_AUTH_IOC_MAGIC	0x66
#define DEBUG_AUTH_SET_CERT	_IOW(DEBUG_AUTH_IOC_MAGIC, 1, struct oplus_debug_cert)
#define DEBUG_AUTH_GET_TOKEN	_IOR(DEBUG_AUTH_IOC_MAGIC, 2, uint8_t[OPLUS_DEBUG_TOKEN_SIZE])
#define DEBUG_AUTH_EXIT		_IOW(DEBUG_AUTH_IOC_MAGIC, 3, uint8_t[OPLUS_DEBUG_TOKEN_SIZE])

static long oplus_debug_auth_dev_ioctl(struct file *filp, unsigned int cmd,
				       unsigned long arg)
{
	struct oplus_debug_auth *oda = filp->private_data;
	void __user *argp = (void __user *)arg;
	int rc;

	switch (cmd) {
	case DEBUG_AUTH_SET_CERT:
		rc = oplus_debug_auth_set_cert(oda, argp);
		if (rc)
			return rc;
		break;
	case DEBUG_AUTH_GET_TOKEN:
		rc = oplus_debug_auth_get_token(oda, argp);
		if (rc)
			return rc;
		break;
	case DEBUG_AUTH_EXIT:
		rc = oplus_debug_auth_user_exit(oda, argp);
		if (rc)
			return rc;
		break;
	default:
		chg_err("bad ioctl %u\n", cmd);
		return -EINVAL;
	}

	return 0;
}

static const struct file_operations oplus_debug_auth_dev_fops = {
	.owner			= THIS_MODULE,
	.llseek			= noop_llseek,
	.open			= oplus_debug_auth_dev_open,
	.unlocked_ioctl		= oplus_debug_auth_dev_ioctl,
};

static int oplus_debug_auth_dev_reg(struct oplus_debug_auth *oda)
{
	int rc;

	oda->auth_dev.minor = MISC_DYNAMIC_MINOR;
	oda->auth_dev.name = "debug_auth";
	oda->auth_dev.fops = &oplus_debug_auth_dev_fops;
	rc = misc_register(&oda->auth_dev);
	if (rc)
		chg_err("misc_register failed, rc=%d\n", rc);

	return rc;
}

ssize_t oplus_debug_auth_get_data(const char *buf, size_t size, const char **ret_buf)
{
	struct oplus_debug_data_header *header;

	if (buf == NULL || ret_buf == NULL || g_oda == NULL)
		return -EINVAL;

	if (size < sizeof(struct oplus_debug_data_header)) {
		chg_err("buf size too small: size=%zu, header_size=%zu\n",
			size, sizeof(struct oplus_debug_data_header));
		return -EINVAL;
	}

	header = (struct oplus_debug_data_header *)buf;
	if (header->magic != OPLUS_DEBUG_DATA_MAGIC) {
		chg_err("invalid data magic\n");
		return -EINVAL;
	}

	mutex_lock(&g_oda->auth_lock);
	if (!g_oda->token_valid) {
		chg_err("token not valid\n");
		mutex_unlock(&g_oda->auth_lock);
		return -EACCES;
	}
	if (memcmp(header->token, g_oda->token, OPLUS_DEBUG_TOKEN_SIZE)) {
		chg_err("token mismatch\n");
		mutex_unlock(&g_oda->auth_lock);
		return -EACCES;
	}
	mutex_unlock(&g_oda->auth_lock);
	if (size - sizeof(struct oplus_debug_data_header) < header->size) {
		chg_err("data size too small\n");
		return -EINVAL;
	}

	cancel_delayed_work_sync(&g_oda->clear_token_work);
	schedule_delayed_work(&g_oda->clear_token_work, msecs_to_jiffies(OPLUS_DEBUG_TOKEN_VALID_TIME_MS));

	*ret_buf = buf + sizeof(struct oplus_debug_data_header);

	return header->size;
}

bool oplus_debug_auth_is_cert_valid(void)
{
	time64_t now;

	if (!g_oda)
		return false;

	mutex_lock(&g_oda->auth_lock);
	if (!g_oda->cert_valid) {
		goto err;
	}
	now = ktime_get_real_seconds();
	if (now < g_oda->cert.time || now > g_oda->cert.time + OPLUS_DEBUG_CERT_VALID_TIME_S) {
		cancel_delayed_work_sync(&g_oda->clear_token_work);
		schedule_delayed_work(&g_oda->clear_token_work, 0);
		goto err;
	}
	mutex_unlock(&g_oda->auth_lock);

	return true;
err:
	mutex_unlock(&g_oda->auth_lock);
	chg_err("%s: cert expired\n", g_oda->user_id);
	return false;
}

static __init int oplus_debug_auth_init(void)
{
	int rc;

	g_oda = kzalloc(sizeof(struct oplus_debug_auth), GFP_KERNEL);
	if (!g_oda)
		return 0;

	mutex_init(&g_oda->auth_lock);
	INIT_DELAYED_WORK(&g_oda->clear_token_work, oplus_debug_auth_clear_token_work);

	rc = oplus_debug_auth_dev_reg(g_oda);
	if (rc)
		goto dev_err;

	return 0;

dev_err:
	mutex_destroy(&g_oda->auth_lock);
	kfree(g_oda);
	g_oda = NULL;
	return 0;
}

static __exit void oplus_debug_auth_exit(void)
{
	if (!g_oda)
		return;

	misc_deregister(&g_oda->auth_dev);
	cancel_delayed_work_sync(&g_oda->clear_token_work);
	mutex_destroy(&g_oda->auth_lock);
	kfree(g_oda);
	g_oda = NULL;
}

oplus_chg_module_late_register(oplus_debug_auth);
