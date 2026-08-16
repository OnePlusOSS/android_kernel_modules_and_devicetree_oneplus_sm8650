#include "pogo_keyboard.h"
#include "owb.h"
#include <linux/firmware.h>

extern int dfu_boot;
extern int tp_ota_status;
extern int max_disconnect_count;

struct dfu_ota_data {
    u32 checksum;
    u32 start_addr;
    u32 file_len;
    int type;
    unsigned char fw_ver[KBVER_LEN_MAX];
};
struct dfu_ota_data *bat_data = NULL;
struct dfu_ota_data *bin_data = NULL;
struct dfu_ota_data *tp_data = NULL;

static int handle_firmware_validation(const struct firmware **fw_entry_ptr,
                                     u32 *fw_data_count, const unsigned char **firmware_data,
                                     u32 *checksum, int *version);
static int handle_dfu_ota_update(const unsigned char *firmware_data, u32 fw_data_count);
static int handle_standard_ota_update(const unsigned char *firmware_data, u32 fw_data_count,
                                     u32 checksum, int version);
static int handle_tp_ota_update(const unsigned char *firmware_data, u32 fw_data_count);
static int handle_kb_ota_update(const unsigned char *firmware_data, u32 fw_data_count);
static void cleanup_resources(const struct firmware *fw_entry);
static void cleanup_dfu_data(void);

// calulator crc32
static u32 dfu_crc32(u8 const * p_data, size_t size, u32 const * p_crc)
{
    u32 crc = 0;
    u32 i = 0;
    crc = (p_crc == NULL) ? 0xFFFFFFFF : ~(*p_crc);
    for (i = 0; i < size; i++){
        crc = crc ^ p_data[i];
        for (u32 j = 8; j > 0; j--) crc = (crc >> 1) ^ (0xEDB88320U & ((crc & 1) ? 0xFFFFFFFF : 0));
    }
    return ~crc;
}

const struct firmware *get_fw_firmware(struct pogo_keyboard_data *pogo_data, const char *patch)
{
    struct platform_device *pdev = pogo_data->plat_dev;
    char *fw_patch = NULL;
    int retry = 2;
    int ret = 0;
    const struct firmware *fw_entry = NULL;

    fw_patch = kzalloc(MAX_FW_NAME_LENGTH, GFP_KERNEL);
    if(fw_patch == NULL)
        return NULL;

    snprintf(fw_patch, MAX_FW_NAME_LENGTH, "%s", patch);
    kb_info("%s %d fw_path is :%s\n", __func__, __LINE__, fw_patch);
    do {
        ret = request_firmware(&fw_entry, fw_patch, &pdev->dev);
        if(ret < 0) {
            kb_err("%s %d Failed to request fw\n", __func__, __LINE__);
            msleep(100);
        } else {
            break;
        }
    } while ((ret < 0) && (--retry > 0));

    kb_info("%s %d fw_path is :%s\n", __func__, __LINE__, fw_patch);
    kfree(fw_patch);
    return fw_entry;
}

static int kpd_fw_isvalid(const unsigned char *fw_data, u32 count, u32 *checksum, int *version)
{
    u32 start_addr = 0;
    u32 get_ver_addr = 0;
    u32 get_chechsum_addr = 0;
    u32 get_checksum = 0;
    u32 add_checksum = 0;
    u32 index = 0;
    u32 i = 0;

    start_addr = pogo_keyboard_client->ota_start_addr;
    get_ver_addr = pogo_keyboard_client->ota_get_version_addr;
    index = pogo_keyboard_client->ota_send_data_start_addr;
    i = index;
    get_chechsum_addr = get_ver_addr - 4;
    kb_debug("%s %d start_addr:0x%x, get_ver_addr:0x%x, ota_send_data_start_addr:0x%x\n",
            __func__, __LINE__, start_addr, get_ver_addr, index);
    if (count < get_ver_addr | count < index) {
        kb_err("%s %d ota file count is too small,please check!!!\n", __func__, __LINE__);
        return -EINVAL;
    }
    do {
        add_checksum += (u32)fw_data[i];
    } while (++i < count);

    get_checksum = (u32)fw_data[get_chechsum_addr] |
                   (u32)fw_data[get_chechsum_addr + 1] << 8 |
                   (u32)fw_data[get_chechsum_addr + 2] << 16 |
                   (u32)fw_data[get_chechsum_addr + 3] << 24;
    if (get_checksum != add_checksum) {
        kb_debug("%s %d add_checksum:0x%08x, get_checksum:0x%08x\n", __func__, __LINE__, add_checksum, get_checksum);
        kb_err("%s %d ota file checksum is not right,please check!!!\n", __func__, __LINE__);
        return -EINVAL;
    }
    *checksum = get_checksum;

    pogo_keyboard_client->kpdmcu_fw_data_ver = (int)fw_data[get_ver_addr + 1] << 8 |
                                                (int)fw_data[get_ver_addr];
    *version = pogo_keyboard_client->kpdmcu_fw_data_ver;
    if (pogo_keyboard_client->kpdmcu_mcu_version < pogo_keyboard_client->kpdmcu_fw_data_ver) {
        pogo_keyboard_client->is_kpdmcu_need_fw_update = true;
    } else {
        pogo_keyboard_client->is_kpdmcu_need_fw_update = false;
    }
    kb_debug("%s %d, fw verison is 0x%04x\n", __func__, __LINE__, *version);
    return 0;
}


/*OTA update online
1.read mcu keyboard version
2.send ota file infomation
3.send ota file datas
4.send ota end infomation
5.ota end reset mcu
*/

static int pogo_keyboard_mcu_version(void)
{
    int ret = 0;
    char write_buf[] = { ONE_WIRE_BUS_PACKET_OTA_CMD, 0x03, 0x01, 0x01, 0x01};
    char buf[] = { ONE_WIRE_BUS_PACKET_OTA_ACK_CMD, 0x06, 0x01, 0x04};
    char read_buf[255] = { 0 };
    int read_len = 0;
    int i = 0;

    for (i = 0; i < 3; i++) {
        ret = pogo_keyboard_write_and_read(write_buf, sizeof(write_buf), read_buf, &read_len);
        if (ret) {
            continue;
        }
        if (memcmp(read_buf, buf, sizeof(buf)) == 0) {
            pogo_keyboard_client->kpdmcu_mcu_version = (read_buf[5] << 8) | read_buf[4];
            kb_debug("%s %d, kpdmcu_mcu_version:0x%x\n", __func__, __LINE__, pogo_keyboard_client->kpdmcu_mcu_version);
            break;
        } else {
            ret = -EINVAL;
        }
    }
    if (i >= 3) {
        kb_err("%s %d err ret:0x%02x\n", __func__, __LINE__, ret);
        pogo_keyboard_client->kpdmcu_mcu_version = 0;
        return ret;
    }
    return 0;
}

static int pogo_keyboard_ota_start_end(u32 len, u32 start_addr, u32 checksum, int version, bool start)
{
    int ret = 0;
    char write_buf[18] = { ONE_WIRE_BUS_PACKET_OTA_CMD, 0x10, 0x02, 0x0E};
    char buf[5] = { ONE_WIRE_BUS_PACKET_OTA_ACK_CMD, 0x04, 0x02, 0x02, 0x00};
    char read_buf[255] = { 0 };
    int read_len = 0;
    int i = 0;

    pm_stay_awake(&pogo_keyboard_client->plat_dev->dev);
    //ack len: ota start 0x04, ota end 0x07
    buf[1] = start ? 0x04 : 0x0b;
    //sub cmd : ota start 0x02, ota end 0x04
    write_buf[2] = start ? 0x02 : 0x04;
    buf[2] = start ? 0x02 : 0x04;
    //ack data len: ota start 0x02, ota end 0x05
    buf[3] = start ? 0x02 : 0x09;
    //ota file info data: 4byte len + 4byte start address + 1byte checksum + 2byte version
    write_buf[4] = (char)(len & 0xff);
    write_buf[5] = (char)((len & 0xff00) >> 8);
    write_buf[6] = (char)((len & 0xff0000) >> 16);
    write_buf[7] = (char)((len & 0xff000000) >> 24);
    write_buf[8] = (char)(start_addr & 0xff);
    write_buf[9] = (char)((start_addr & 0xff00) >> 8);
    write_buf[10] = (char)((start_addr & 0xff0000) >> 16);
    write_buf[11] = (char)((start_addr & 0xff000000) >> 24);
    write_buf[12] = (char)(checksum & 0xff);
    write_buf[13] = (char)((checksum & 0xff00) >> 8);
    write_buf[14] = (char)((checksum & 0xff0000) >> 16);
    write_buf[15] = (char)((checksum & 0xff000000) >> 24);
    write_buf[16] = (char)(version & 0xff);
    write_buf[17] = (char)((version & 0xff00) >> 8);

    for (i = 0; i < 3; i++) {
        ret = pogo_keyboard_write(write_buf, sizeof(write_buf));
        if (ret) {
            mdelay(50);
            continue;
        }
        mdelay(200);
        ret = pogo_keyboard_read(read_buf, &read_len);
        if (ret == 0) {
            if (memcmp(read_buf, buf, sizeof(buf)) == 0) {
                kb_debug("%s %d send ota file infomation success.\n", __func__, __LINE__);
                break;
            } else {
                kb_err("%s %d, read status:%d\n", __func__, __LINE__, read_buf[4]);
                ret = -EINVAL;
            }
        } else {
            mdelay(50);
            continue;
        }
    }
    if (i >= 3) {
        kb_err("%s %d err ret:0x%02x\n", __func__, __LINE__, ret);
        return ret;
    }
    pm_relax(&pogo_keyboard_client->plat_dev->dev);
    return 0;
}

static int pogo_keyboard_ota_write_datas(const unsigned char *fw_data, u32 len)
{
    int ret = 0;
    int i = 0;
    int j = 0;
    char write_buf[ONE_WRITY_LEN_MAX + 11] =
            { ONE_WIRE_BUS_PACKET_OTA_CMD, ONE_WRITY_LEN_MAX + 9, 0x03, ONE_WRITY_LEN_MAX + 7};
    char buf[11] = { ONE_WIRE_BUS_PACKET_OTA_ACK_CMD, 0x09, 0x03, 0x07};
    char read_buf[255] = { 0 };
    int read_len = 0;
    u32 write_len = 0, fw_len = 0, fw_offset = 0;
    char send_rx[ONE_WRITY_LEN_MAX] = {0};
    int package_index = 0;

    fw_len = len;
    while (fw_len) {
        write_len = (fw_len < ONE_WRITY_LEN_MAX) ? fw_len : ONE_WRITY_LEN_MAX;
        kb_debug("%s %d write_len:0x%08x\n", __func__, __LINE__, write_len);
        for(i = 0; i < write_len; i++) {
            send_rx[i] = fw_data[fw_offset + i];
        }
        if(fw_len < ONE_WRITY_LEN_MAX) {
            for(i = 0; i < ONE_WRITY_LEN_MAX - fw_len; i++) {
                //fill one package with 0x00
                send_rx[ONE_WRITY_LEN_MAX - i -1] = 0x00;
            }
        }
        write_buf[1] = (fw_len < ONE_WRITY_LEN_MAX) ? fw_len + 9 : ONE_WRITY_LEN_MAX + 9;
        write_buf[3] = (fw_len < ONE_WRITY_LEN_MAX) ? fw_len + 7 : ONE_WRITY_LEN_MAX + 7;
        //ota  write datas: 4byte offset address + 2byte package index + 1byte len + (n-7)byte datas
        write_buf[4] = (char)(fw_offset & 0xff);
        write_buf[5] = (char)((fw_offset & 0xff00) >> 8);
        write_buf[6] = (char)((fw_offset & 0xff0000) >> 16);
        write_buf[7] = (char)((fw_offset & 0xff000000) >> 24);
        write_buf[8] = (char)(package_index & 0xff);
        write_buf[9] = (char)((package_index & 0xff00) >> 8);
        write_buf[10] = write_len;
        memcpy(&write_buf[11], send_rx, sizeof(send_rx));

        for (j = 0; j < 3; j++) {
            ret = pogo_keyboard_write_and_read(write_buf, sizeof(write_buf), read_buf, &read_len);
            if (ret) {
                continue;
            }
            if (memcmp(read_buf, buf, 4) == 0) {
                if ((read_buf[4] == 0) &&
                    ((memcmp(&read_buf[5], &write_buf[8], 2)) == 0) &&
                    ((memcmp(&read_buf[7], &write_buf[4], 4)) == 0)) {
                    kb_debug("%s %d send ota data success, package_index:%d, fw_offset:0x%08x\n",
                        __func__, __LINE__, package_index, fw_offset);
                    break;
                } else {
                    pogo_keyboard_show_buf(read_buf, read_len);
                    kb_debug("%s %d send ota data ack status:%d\n", __func__, __LINE__, read_buf[4]);
                    ret = -EINVAL;
                }
            } else {
                ret = -EINVAL;
            }
        }
        if (j >= 3) {
            kb_err("%s %d err ret:0x%02x\n", __func__, __LINE__, ret);
            return ret;
        }

        package_index++;
        fw_offset += write_len;
        fw_len -= write_len;
        kb_debug("%s %d package_index:%d,fw_offset:0x%08x,fw_len:0x%08x\n", __func__, __LINE__, package_index, fw_offset, fw_len);
        if(pogo_keyboard_client->fw_update_progress < FW_PROGRESS_96 * FW_PERCENTAGE_100) {
            pogo_keyboard_client->fw_update_progress = FW_PROGRESS_3 * FW_PERCENTAGE_100 +
                FW_PROGRESS_93 * (len - fw_len) * FW_PERCENTAGE_100 / len;
        } else {
            pogo_keyboard_client->fw_update_progress = FW_PROGRESS_96 * FW_PERCENTAGE_100  +
                FW_PROGRESS_93 * (len - fw_len) * FW_PERCENTAGE_100 / len;
        }
    }

    return 0;
}

static int pogo_keyboard_ota_end_reset(void)
{
    int ret = 0;
    char write_buf[] = { ONE_WIRE_BUS_PACKET_OTA_CMD, 0x06, 0x05, 0x04, 0x57, 0x4e, 0x38, 0x30};
    char buf[] = { ONE_WIRE_BUS_PACKET_OTA_ACK_CMD, 0x03, 0x05, 0x01, 0x00};
    char read_buf[255] = { 0 };
    int read_len = 0;
    int i = 0;

    for (i = 0; i < 3; i++) {
        ret = pogo_keyboard_write_and_read(write_buf, sizeof(write_buf), read_buf, &read_len);
        if (ret) {
            continue;
        }
        if (memcmp(read_buf, buf, sizeof(buf)) == 0) {
            kb_debug("%s %d  success!!!\n", __func__, __LINE__);
            break;
        } else {
            kb_debug("%s %d ota reset ack status:%d\n", __func__, __LINE__, read_buf[4]);
            ret = -EINVAL;
        }
    }
    if (i >= 3) {
        kb_err("%s %d err ret:0x%02x\n", __func__, __LINE__, ret);
        return ret;
    }
    return 0;
}

static int kpd_fw_update(const unsigned char *fw_data, u32 count, u32 checksum, int version)
{
    int ret = 0;
    int retry = 3;
    int i = 0;
    u32 start_addr = 0;
    u32 index = 0;
    u32 len = 0;

    start_addr = pogo_keyboard_client->ota_start_addr;
    index = pogo_keyboard_client->ota_send_data_start_addr;
    len =  count - index;
    ret = pogo_keyboard_mcu_version();
    if (ret) {
        kb_err("%s %d get mcu version fail\n", __func__, __LINE__);
        return ret;
    }
    ret = pogo_keyboard_ota_start_end(len, start_addr, checksum, version, 1);
    if (ret) {
        kb_err("%s %d send ota file infomation fail\n", __func__, __LINE__);
        return ret;
    }
    for (i = 0; i < retry; i++)
    {
        ret = pogo_keyboard_ota_write_datas(&fw_data[index], len);
        if (ret) {
            kb_err("%s %d ota write datas fail, retry: %d\n", __func__, __LINE__, i);
        } else {
            kb_debug("%s %d ota write datas success!!!\n", __func__, __LINE__);
            break;
        }
    }
    if (i >= retry) {
        kb_err("%s %d ota write datas fail\n", __func__, __LINE__);
        return ret;
    }
    ret = pogo_keyboard_ota_start_end(len, start_addr, checksum, version, 0);
    if (ret) {
        kb_err("%s %d send ota end infomation fail\n", __func__, __LINE__);
        return ret;
    }

    ret = pogo_keyboard_ota_end_reset();
    if (ret) {
        kb_err("%s %d send ota end reset fail\n", __func__, __LINE__);
        return ret;
    }

    return ret;
}

/*
dfu ota thread
1.send bat file
2.send TP file, do tp ota
3.send bat file, do kb ota
basic info: 64byte
fw info: (64byte)
         ID,file type(2byte),
         offset,file start address(4byte)
         length,file lenghth(4byte)
         digest,file digest(32byte)
         vesion len(1byte)
         version(Nbyte)
         reverse
*/
static inline struct dfu_ota_data *get_fw_data(const unsigned char *fw_data, u32 addr, int ver_len)
{
    unsigned char fwinfo[DFU_FW_INFO_LEN] = {0};
    struct dfu_ota_data *data = NULL;
    u32 crc32 = 0;
    u32 i = 0;

    data = kzalloc(sizeof(*data), GFP_KERNEL);
    if (!data) {
        kb_err("%s %d kzalloc err\n", __func__, __LINE__);
        return 0;
    }

	memcpy(fwinfo, &fw_data[addr], sizeof(fwinfo));
    data->start_addr = fwinfo[2] << 24 | fwinfo[3] << 16 | fwinfo[4] << 8 | fwinfo[5];
    data->file_len = fwinfo[6] << 24 | fwinfo[7] << 16 | fwinfo[8] << 8 | fwinfo[9];
    data->type = fwinfo[0] << 8 | fwinfo[1];
    //kb version format: KA030_A_1.0.4_XXX, 43+9-1
    //tp version format: TP_A_02 (54 50 5f 41 5f 30 32)(ascll)
    kb_debug("%s %d start_addr: 0x%x,file_len:0x%x, type:0x%x\n", __func__, __LINE__,
        data->start_addr, data->file_len, data->type);
    memset(data->fw_ver, 0, ver_len);
    memcpy(data->fw_ver, &fwinfo[43], ver_len);
    kb_debug("%s %d\n", __func__, __LINE__);
    if (data->type == 0x04) {//tp file
        do {
            crc32 += (u32)fw_data[data->start_addr + i];
        } while (++i < data->file_len);
    } else {
        crc32 = dfu_crc32(&fw_data[data->start_addr], data->file_len, &crc32);
    }
    data->checksum = crc32;

    kb_debug("%s %d start_addr: 0x%x,file_len:0x%x, type:0x%x, checksum: 0x%x, %s\n", __func__, __LINE__,
        data->start_addr, data->file_len, data->type, data->checksum, data->fw_ver);
    return data;
}

static void cleanup_dfu_data(void)
{
    if (bat_data) {
        kfree(bat_data);
        bat_data = NULL;
    }
    if (bin_data) {
        kfree(bin_data);
        bin_data = NULL;
    }
    if (tp_data) {
        kfree(tp_data);
        tp_data = NULL;
    }
}

static int kpd_fw_dfu_isvalid(const unsigned char *fw_data, u32 count)
{
    u32 fwinfo_addr = 0;

    if (DFU_FW_INFO_LEN < 64) {
        kb_err("%s %d DFU_FW_INFO_LEN define too small,please check!!!\n", __func__, __LINE__);
        return -EINVAL;
    }

    if (count < DFU_FW_INFO_LEN * 3) {
        kb_err("%s %d ota file count is too small,please check!!!\n", __func__, __LINE__);
        return -EINVAL;
    }
    fwinfo_addr = pogo_keyboard_client->dfu_fwinfo_start_addr;

    bat_data = get_fw_data(fw_data, fwinfo_addr, 13);
    if (!bat_data || (bat_data->start_addr + bat_data->file_len) > count) {
        kb_err("%s %d bat_data is NULL, or ota file count < bat_start_addr + bat_len, please check!!!\n", __func__, __LINE__);
        return -EINVAL;
    }
    bin_data = get_fw_data(fw_data, fwinfo_addr + DFU_FW_INFO_LEN, 13);
    if (!bin_data || (bin_data->start_addr + bin_data->file_len) > count) {
        kb_err("%s %d bin_data is NULL, or ota file count < bin_start_addr + bin_len, please check!!!\n", __func__, __LINE__);
        return -EINVAL;
    }

    tp_data = get_fw_data(fw_data, fwinfo_addr + DFU_FW_INFO_LEN * 2, 7);
    if (!tp_data || (tp_data->start_addr + tp_data->file_len) > count) {
        kb_err("%s %d tp_data is NULL, or ota file count < tp_start_addr + tp_len, please check!!!\n", __func__, __LINE__);
        return -EINVAL;
    }

    //version format: KA030_A_1.0.4_XXX, 43+9-1
    if (memcmp(bat_data->fw_ver, bat_data->fw_ver, 13)) {
        kb_err("%s %d bat_fw_version != bin_fw_version, please check!!!\n", __func__, __LINE__);
        return -EINVAL;
    }
    pogo_keyboard_client->kpdmcu_fw_data_ver =
                (int)(bat_data->fw_ver[8] & 0x0f) << 8 |
                (int)(bat_data->fw_ver[10] & 0x0f) << 4 |
                (int)(bat_data->fw_ver[12] & 0x0f);
    kb_debug("%s %d kpdmcu_fw_data_ver: 0x%x\n", __func__, __LINE__, pogo_keyboard_client->kpdmcu_fw_data_ver);

    //get tp version: TP_A_02 (54 50 5f 41 5f 30 32)(ascll)
    memset(pogo_keyboard_client->report_tpver, 0, sizeof(pogo_keyboard_client->report_tpver));
    memcpy(pogo_keyboard_client->report_tpver, tp_data->fw_ver,
            sizeof(pogo_keyboard_client->report_tpver));

    kb_debug("%s %d tp_ver: 0x%x\n", __func__, __LINE__, tp_data->fw_ver[6] );
    if ((pogo_keyboard_client->kpdmcu_mcu_version < pogo_keyboard_client->kpdmcu_fw_data_ver) ||
       (tp_data->fw_ver[6] == 0x43)) {//1.0.7_TP_A_0C not support tp ota
        pogo_keyboard_client->is_kpdmcu_need_fw_update = true;
    } else {
        pogo_keyboard_client->is_kpdmcu_need_fw_update = false;
    }
    return 0;
}

static int pogo_keyboard_dfu_start_end(u32 len, u32 start_addr, u32 checksum, int type, bool start)
{
    int ret = 0;
    char write_buf[18] = { ONE_WIRE_BUS_PACKET_OTA_CMD, 0x10, 0x02, 0x0E};
    char buf[5] = { ONE_WIRE_BUS_PACKET_OTA_ACK_CMD, 0x04, 0x02, 0x02, 0x00};
    char read_buf[255] = { 0 };
    int read_len = 0;
    int i = 0;
    int j = 0;

    pm_stay_awake(&pogo_keyboard_client->plat_dev->dev);
    //ack len: ota start 0x04, ota end 0x07
    buf[1] = start ? 0x04 : 0x07;
    //sub cmd : ota start 0x02, ota end 0x04
    write_buf[2] = start ? 0x02 : 0x04;
    buf[2] = start ? 0x02 : 0x04;
    //ack data len: ota start 0x02, ota end 0x05
    buf[3] = start ? 0x02 : 0x05;
    //ota file info data: 4byte len + 4byte start address + 1byte checksum + 2byte type
    write_buf[4] = (char)(len & 0xff);
    write_buf[5] = (char)((len & 0xff00) >> 8);
    write_buf[6] = (char)((len & 0xff0000) >> 16);
    write_buf[7] = (char)((len & 0xff000000) >> 24);
    write_buf[8] = (char)(start_addr & 0xff);
    write_buf[9] = (char)((start_addr & 0xff00) >> 8);
    write_buf[10] = (char)((start_addr & 0xff0000) >> 16);
    write_buf[11] = (char)((start_addr & 0xff000000) >> 24);
    write_buf[12] = (char)(checksum & 0xff);
    write_buf[13] = (char)((checksum & 0xff00) >> 8);
    write_buf[14] = (char)((checksum & 0xff0000) >> 16);
    write_buf[15] = (char)((checksum & 0xff000000) >> 24);
    write_buf[16] = (char)(type & 0xff);
    write_buf[17] = (char)((type & 0xff00) >> 8);

    for (i = 0; i < 3; i++) {
        ret = pogo_keyboard_write(write_buf, sizeof(write_buf));
        if (ret) {
            mdelay(50);
            continue;
        }
        do {
            mdelay(100);
            ret = pogo_keyboard_read(read_buf, &read_len);
            kb_err("%s %d, ret:0x%02x\n", __func__, __LINE__, ret);
            if (ret == 0 && memcmp(read_buf, buf, sizeof(buf)) == 0) {
                break;
            } else {
                j++;
            }
        } while (j < 4);
        if (j >= 4) {
            kb_err("%s %d, read status:%d\n", __func__, __LINE__, read_buf[4]);
            ret = -EINVAL;
            continue;
        } else {
            kb_debug("%s %d send ota file infomation success.\n", __func__, __LINE__);
            break;
        }
    }
    if (i >= 3) {
        kb_err("%s %d err ret:0x%02x\n", __func__, __LINE__, ret);
        return ret;
    }
    pm_relax(&pogo_keyboard_client->plat_dev->dev);
    return 0;
}

static int pogo_keyboard_dfu_write_datas(const unsigned char *fw_data, u32 len)
{
    int ret = 0;
    int i = 0;
    int j = 0;
    u32 crc32 = 0;
    char write_buf[DFU_ONE_WRITY_LEN_MAX + 11] =
            { ONE_WIRE_BUS_PACKET_OTA_CMD, DFU_ONE_WRITY_LEN_MAX + 9, 0x03, DFU_ONE_WRITY_LEN_MAX + 7};
    char buf[11] = { ONE_WIRE_BUS_PACKET_OTA_ACK_CMD, 0x09, 0x03, 0x07};
    char read_buf[255] = { 0 };
    int read_len = 0;
    u32 write_len = 0, fw_len = 0, fw_offset = 0;
    char send_rx[DFU_ONE_WRITY_LEN_MAX] = {0};
    u8 package_index = 0;

    fw_len = len;
    while (fw_len) {
        write_len = (fw_len < DFU_ONE_WRITY_LEN_MAX) ? fw_len : DFU_ONE_WRITY_LEN_MAX;;
        for(i = 0; i < write_len; i++) {
            send_rx[i] = fw_data[fw_offset + i];
        }
        if(fw_len < DFU_ONE_WRITY_LEN_MAX) {
            for(i = 0; i < DFU_ONE_WRITY_LEN_MAX - fw_len; i++) {
                //fill one package with 0x00
                send_rx[DFU_ONE_WRITY_LEN_MAX - i -1] = 0x00;
            }
        }
        memcpy(&write_buf[11], send_rx, sizeof(send_rx));
        crc32 = dfu_crc32(&write_buf[11], write_len, &crc32);
        write_buf[1] = (fw_len < DFU_ONE_WRITY_LEN_MAX) ? fw_len + 9 : DFU_ONE_WRITY_LEN_MAX + 9;
        write_buf[3] = (fw_len < DFU_ONE_WRITY_LEN_MAX) ? fw_len + 7 : DFU_ONE_WRITY_LEN_MAX + 7;
        //ota  write datas: 4byte crc32 + 2byte package index + 1byte len + (n-7)byte datas
        write_buf[4] = (char)(crc32 & 0xff);
        write_buf[5] = (char)((crc32 & 0xff00) >> 8);
        write_buf[6] = (char)((crc32 & 0xff0000) >> 16);
        write_buf[7] = (char)((crc32 & 0xff000000) >> 24);
        write_buf[8] = (char)(package_index & 0xff);
        write_buf[9] = 0x00;
        write_buf[10] = write_len;

        for (j = 0; j < 3; j++) {
            ret = pogo_keyboard_write_and_read(write_buf, sizeof(write_buf), read_buf, &read_len);
            if (ret) {
                continue;
            }
            if (memcmp(read_buf, buf, 4) == 0) {
                if (read_buf[4] == 0 &&
                    ((memcmp(&read_buf[5], &write_buf[8], 1)) == 0)/* &&
                    ((memcmp(&read_buf[7], &write_buf[4], 4)) == 0)*/) {
                    kb_debug("%s %d send ota data success, package_index:%d, fw_offset:0x%08x\n",
                        __func__, __LINE__, package_index, fw_offset);
                    break;
                } else {
                    pogo_keyboard_show_buf(read_buf, read_len);
                    kb_debug("%s %d send ota data ack status:%d\n", __func__, __LINE__, read_buf[4]);
                    ret = -EINVAL;
                }
            } else {
                ret = -EINVAL;
            }
        }

        if (j >= 3) {
            kb_err("%s %d err ret:0x%02x\n", __func__, __LINE__, ret);
            return ret;
        }

        package_index++;
        fw_offset += write_len;
        fw_len -= write_len;
        kb_debug("%s %d package_index:%d,fw_offset:0x%08x,fw_len:0x%08x\n", __func__, __LINE__, package_index, fw_offset, fw_len);
        if(pogo_keyboard_client->fw_update_progress < FW_PROGRESS_5 * FW_PERCENTAGE_100) {
            pogo_keyboard_client->fw_update_progress = FW_PROGRESS_3 * FW_PERCENTAGE_100 +
                FW_PROGRESS_2 * (len - fw_len) * FW_PERCENTAGE_100 / len;
        } else if(pogo_keyboard_client->fw_update_progress < FW_PROGRESS_50 * FW_PERCENTAGE_100) {
            pogo_keyboard_client->fw_update_progress = FW_PROGRESS_5 * FW_PERCENTAGE_100 +
                FW_PROGRESS_45 * (len - fw_len) * FW_PERCENTAGE_100 / len;
        } else if(pogo_keyboard_client->fw_update_progress < FW_PROGRESS_96 * FW_PERCENTAGE_100) {
            pogo_keyboard_client->fw_update_progress = FW_PROGRESS_50 * FW_PERCENTAGE_100 +
                FW_PROGRESS_46 * (len - fw_len) * FW_PERCENTAGE_100 / len;
        } else {
            pogo_keyboard_client->fw_update_progress = FW_PROGRESS_96 * FW_PERCENTAGE_100;
        }
    }

    return 0;
}

static int kpd_fw_dfu_update(const unsigned char *fw_data, u32 count, u32 checksum, u32 addr, int len, int type)
{
    int ret = 0;
    int retry = 3;
    int i = 0;

    ret = pogo_keyboard_dfu_start_end(len, 0, 0, type, 1);
    if (ret) {
        kb_err("%s %d send ota file infomation fail\n", __func__, __LINE__);
        return ret;
    }

    for (i = 0; i < retry; i++)
    {
        ret = pogo_keyboard_dfu_write_datas(&fw_data[addr], len);
        if (ret) {
            kb_err("%s %d ota write datas fail, retry: %d\n", __func__, __LINE__, i);
        } else {
            kb_err("%s %d ota write datas success!!!\n", __func__, __LINE__);
            break;
        }
    }

    if (i >= retry) {
        kb_err("%s %d ota write datas fail\n", __func__, __LINE__);
        return ret;
    }
    msleep(50);
    //send TP data or KB data end,set heartbeat timeout to 2s
    if (type != 1) {
        max_disconnect_count = DFU_DISCONNECT_COUNT; //2s
    }
    ret = pogo_keyboard_dfu_start_end(len, 0, checksum, type, 0);
    if (ret) {
        kb_err("%s %d send ota end infomation fail\n", __func__, __LINE__);
        return ret;
    }
    return ret;
}

static int tp_dfu_start(void)
{
    int ret = 0;
    char write_buf[4 + TPVER_LEN] = { ONE_WIRE_BUS_PACKET_USER_GENERAL_CMD, TPVER_LEN + 2, 0x12, TPVER_LEN};
    char buf[4] = { ONE_WIRE_BUS_PACKET_USER_GENERAL_ACK_CMD, 0x03, 0x12, 0x01};
    char read_buf[255] = { 0 };
    int read_len = 0;
    int i = 0;
    int j = 0;

    pm_stay_awake(&pogo_keyboard_client->plat_dev->dev);
    memcpy(&write_buf[4], pogo_keyboard_client->report_tpver, TPVER_LEN);

    for (i = 0; i < 3; i++) {
        ret = pogo_keyboard_write(write_buf, sizeof(write_buf));
        if (ret) {
            mdelay(50);
            continue;
        }
        do {
            mdelay(100);
            ret = pogo_keyboard_read(read_buf, &read_len);
            kb_err("%s %d, ret:0x%02x\n", __func__, __LINE__, ret);
            if (ret == 0 && memcmp(read_buf, buf, sizeof(buf)) == 0) {
                break;
            } else {
                j++;
            }
        } while (j < 4);
        if (j >= 4) {
            kb_err("%s %d, read status:%d\n", __func__, __LINE__, read_buf[4]);
            ret = -EINVAL;
            continue;
        } else {
            kb_debug("%s %d send ota file infomation success.\n", __func__, __LINE__);
            break;
        }
    }
    if (i >= 3) {
        kb_err("%s %d err ret:0x%02x\n", __func__, __LINE__, ret);
        return ret;
    }
    ret = read_buf[4];
    pm_relax(&pogo_keyboard_client->plat_dev->dev);
    return ret;
}

static int tp_dfu_write_datas(const unsigned char *fw_data, u32 len, int *count)
{
    int ret = 0;
    int i = 0;
    int j = 0;
    char write_buf[DFU_ONE_WRITY_LEN_MAX + 6] =
            {ONE_WIRE_BUS_PACKET_USER_GENERAL_CMD, DFU_ONE_WRITY_LEN_MAX + 4, 0x13, DFU_ONE_WRITY_LEN_MAX + 2};
    char buf[4] = { ONE_WIRE_BUS_PACKET_USER_GENERAL_ACK_CMD, 0x05, 0x13, 0x03};
    char read_buf[255] = { 0 };
    int read_len = 0;
    u32 write_len = 0, fw_len = 0, fw_offset = 0;
    char send_rx[DFU_ONE_WRITY_LEN_MAX] = {0};
    int package_index = 0x0000;

    fw_len = len;
    while (fw_len) {
        package_index++;
        write_len = (fw_len < DFU_ONE_WRITY_LEN_MAX) ? fw_len : DFU_ONE_WRITY_LEN_MAX;;
        for(i = 0; i < write_len; i++) {
            send_rx[i] = fw_data[fw_offset + i];
        }
        if(fw_len < DFU_ONE_WRITY_LEN_MAX) {
            for(i = 0; i < DFU_ONE_WRITY_LEN_MAX - fw_len; i++) {
                //fill one package with 0x00
                send_rx[DFU_ONE_WRITY_LEN_MAX - i -1] = 0x00;
            }
        }
        write_buf[4] = (char)(package_index & 0xff);
        write_buf[5] = (char)((package_index & 0xff00) >> 8);
        memcpy(&write_buf[6], send_rx, sizeof(send_rx));

        for (j = 0; j < 3; j++) {
            ret = pogo_keyboard_write_and_read(write_buf, sizeof(write_buf), read_buf, &read_len);
            if (ret) {
                continue;
            }
            if (memcmp(read_buf, buf, sizeof(buf)) == 0) {
                if ((read_buf[6] == 0x01) || (read_buf[6] == 0x03)) {
                    kb_debug("%s %d send ota data success, package_index:%d, fw_offset:0x%08x\n",
                        __func__, __LINE__, package_index, fw_offset);
                    break;
                } else {
                    pogo_keyboard_show_buf(read_buf, read_len);
                    kb_debug("%s %d send ota data package_index %d loss:%d\n",
                        __func__, __LINE__, package_index, read_buf[6]);
                    ret = read_buf[6];
                    return ret;
                }
            } else {
                ret = -EINVAL;
            }
        }

        if (j >= 3) {
            kb_err("%s %d err ret:0x%02x\n", __func__, __LINE__, ret);
            return ret;
        }

        fw_offset += write_len;
        fw_len -= write_len;
        kb_debug("%s %d package_index:%d,fw_offset:0x%08x,fw_len:0x%08x\n", __func__, __LINE__, package_index, fw_offset, fw_len);
        if(pogo_keyboard_client->fw_update_progress < FW_PROGRESS_5 * FW_PERCENTAGE_100) {
            pogo_keyboard_client->fw_update_progress = FW_PROGRESS_3 * FW_PERCENTAGE_100 +
                FW_PROGRESS_2 * (len - fw_len) * FW_PERCENTAGE_100 / len;
        } else if(pogo_keyboard_client->fw_update_progress < FW_PROGRESS_25 * FW_PERCENTAGE_100) {
            pogo_keyboard_client->fw_update_progress = FW_PROGRESS_5 * FW_PERCENTAGE_100 +
                FW_PROGRESS_20 * (len - fw_len) * FW_PERCENTAGE_100 / len;
        } else {
            pogo_keyboard_client->fw_update_progress = FW_PROGRESS_25 * FW_PERCENTAGE_100;
        }
    }
    *count = package_index;
    return 0;
}

static int tp_dfu_end(int count, u32 checksum)
{
    int ret = 0;
    char write_buf[10] = { ONE_WIRE_BUS_PACKET_USER_GENERAL_CMD, 0x08, 0x14, 0x06};
    char buf[5] = { ONE_WIRE_BUS_PACKET_USER_GENERAL_ACK_CMD, 0x03, 0x14, 0x01, 0x02};
    char read_buf[255] = { 0 };
    int read_len = 0;
    int i = 0;
    int j = 0;

    pm_stay_awake(&pogo_keyboard_client->plat_dev->dev);
    write_buf[4] = (char)(count & 0xff);
    write_buf[5] = (char)((count & 0xff00) >> 8);
    write_buf[6] = (char)(checksum & 0xff);
    write_buf[7] = (char)((checksum & 0xff00) >> 8);
    write_buf[8] = (char)((checksum & 0xff0000) >> 16);
    write_buf[9] = (char)((checksum & 0xff000000) >> 24);

    for (i = 0; i < 3; i++) {
        ret = pogo_keyboard_write(write_buf, sizeof(write_buf));
        if (ret) {
            mdelay(50);
            continue;
        }
        do {
            mdelay(100);
            ret = pogo_keyboard_read(read_buf, &read_len);
            kb_err("%s %d, ret:0x%02x\n", __func__, __LINE__, ret);
            if (ret == 0 && memcmp(read_buf, buf, sizeof(buf)) == 0) {
                break;
            } else {
                j++;
            }
        } while (j < 4);
        if (j >= 4) {
            kb_err("%s %d, read status:%d\n", __func__, __LINE__, read_buf[4]);
            ret = -EINVAL;
            continue;
        } else {
            kb_debug("%s %d send tp file success.\n", __func__, __LINE__);
            break;
        }
    }
    if (i >= 3) {
        kb_err("%s %d err ret:0x%02x\n", __func__, __LINE__, ret);
        return ret;
    }

    pm_relax(&pogo_keyboard_client->plat_dev->dev);
    return 0;
}

static int tp_fw_dfu_update(const unsigned char *fw_data, u32 count, u32 checksum, u32 addr, int len)
{
    int ret = 0;
    int retry = 3;
    int i = 0;
    int tp_package_count = 0;
    int loss_count = 0;

do_restart:
    ret = tp_dfu_start();
    if (ret < 0) {
        kb_err("%s %d send ota file infomation fail\n", __func__, __LINE__);
        return ret;
    } else if (ret == 0x01) {
        kb_err("%s %d tp version is same,not need update\n", __func__, __LINE__);
        return ret;
    }

    for (i = 0; i < retry; i++)
    {
        ret = tp_dfu_write_datas(&fw_data[addr], len, &tp_package_count);
        if (ret < 0) {
            kb_err("%s %d ota write datas fail, retry: %d\n", __func__, __LINE__, i);
        } else if (ret == 0x02) {//loss package,restart
            kb_err("%s %d ota write datas loss %d\n", __func__, __LINE__, loss_count);
            loss_count++;
            if (loss_count > 3) {
                kb_err("%s %d ota write datas faild\n", __func__, __LINE__);
                return -1;
            }
            i = 0;
            goto do_restart;
        } else {
            kb_err("%s %d ota write datas success!!!\n", __func__, __LINE__);
            break;
        }
    }

    if (i >= retry) {
        kb_err("%s %d ota write datas fail\n", __func__, __LINE__);
        return ret;
    }
    msleep(50);
    max_disconnect_count = DFU_DISCONNECT_COUNT; //2s

    ret = tp_dfu_end(tp_package_count, checksum);
    if (ret) {
        kb_err("%s %d send ota end infomation fail\n", __func__, __LINE__);
        return ret;
    }
    return ret;
}

static int handle_firmware_validation(const struct firmware **fw_entry_ptr,
                                     u32 *fw_data_count, const unsigned char **firmware_data,
                                     u32 *checksum, int *version)
{
    if (pogo_keyboard_client->ota_firmware_name == NULL) {
        kb_err("ota_firmware_name is NULL!!!\n");
        return -EINVAL;
    }

    *fw_entry_ptr = get_fw_firmware(pogo_keyboard_client, pogo_keyboard_client->ota_firmware_name);
    if (*fw_entry_ptr == NULL) {
        kb_err("fw request firmware fail\n");
        return -EINVAL;
    }

    *fw_data_count = (u32)(*fw_entry_ptr)->size;
    *firmware_data = (*fw_entry_ptr)->data;
    kb_info("fw count 0X%x\n", *fw_data_count);
    if (pogo_keyboard_client->pogopin_ota_dfu) {
        return kpd_fw_dfu_isvalid(*firmware_data, *fw_data_count);
    } else {
        return kpd_fw_isvalid(*firmware_data, *fw_data_count, checksum, version);
    }
}

static int handle_tp_ota_success(void)
{
    tp_ota_status = OTA_STATUS_ACTIVE;
    max_disconnect_count = TP_OTA_START_DISCONNECT_COUNT; // 15s

    // TP OTA need > 12s
    do {
        msleep(OTA_SLEEP_INTERVAL);
        pogo_keyboard_client->fw_update_progress += TP_OTA_PROGRESS_INCREMENT;
        kb_debug("fw_update_progress:%d\n", pogo_keyboard_client->fw_update_progress);
    } while ((tp_ota_status == OTA_STATUS_ACTIVE) &&
            (pogo_keyboard_client->fw_update_progress <= FW_PROGRESS_50 * FW_PERCENTAGE_100) &&
            (pogo_keyboard_client->fw_update_progress >= FW_PROGRESS_25 * FW_PERCENTAGE_100));

    if ((pogo_keyboard_client->fw_update_progress < FW_PROGRESS_25 * FW_PERCENTAGE_100) ||
        (pogo_keyboard_client->fw_update_progress > FW_PROGRESS_50 * FW_PERCENTAGE_100)) {
        kb_info("maybe KB plugout or be changed\n");
        pogo_keyboard_client->kpd_fw_status = FW_UPDATE_FAIL;
        pogo_keyboard_client->fw_update_progress = FW_PROGERSS_0;
        tp_ota_status = OTA_STATUS_INACTIVE;
        return -EIO;
    } else if (tp_ota_status == OTA_STATUS_INACTIVE) {
        kb_info("tp ota success!!!\n");
        max_disconnect_count = DEFAULT_DISCONNECT_COUNT;
    }

    return 0;
}

static int handle_tp_ota_failure(void)
{
    kb_err("tp firmware ota fail!!!\n");
    if (pogo_keyboard_client->kpdmcu_mcu_version > KBMCU_VESION_1_0_7) {
        pogo_keyboard_client->kpd_fw_status = FW_UPDATE_FAIL;
        pogo_keyboard_client->fw_update_progress = FW_PROGERSS_0;
        max_disconnect_count = DEFAULT_DISCONNECT_COUNT;
        return -EIO;
    } else {
        kb_info("kb version < 1.0.8, pass tp ota!!!\n");
    }
    return 0;
}

static int handle_tp_ota_update(const unsigned char *firmware_data, u32 fw_data_count)
{
    int ret;

    pogo_keyboard_client->fw_update_progress = FW_PROGRESS_6 * FW_PERCENTAGE_100;
    ret = tp_fw_dfu_update(firmware_data, fw_data_count, tp_data->checksum,
                            tp_data->start_addr, tp_data->file_len);

    if (ret == 0) {
        return handle_tp_ota_success();
    } else if (ret < 0) {
        return handle_tp_ota_failure();
    }

    return 0;
}

static int handle_kb_ota_update(const unsigned char *firmware_data, u32 fw_data_count)
{
    int ret;

    pogo_keyboard_client->fw_update_progress = FW_PROGRESS_50 * FW_PERCENTAGE_100;
    ret = kpd_fw_dfu_update(firmware_data, fw_data_count, bin_data->checksum,
                          bin_data->start_addr, bin_data->file_len, bin_data->type);

    if (ret == 0) {
        max_disconnect_count = DFU_RESET_DISCONNECT_COUNT; // 20s
        dfu_boot = 1;

        // DFU boot need > 6s
        do {
            msleep(OTA_SLEEP_INTERVAL);
            pogo_keyboard_client->fw_update_progress += KB_OTA_PROGRESS_INCREMENT;
            kb_debug("fw_update_progress:%d\n", pogo_keyboard_client->fw_update_progress);
        } while ((pogo_keyboard_client->fw_update_progress <= FW_PROGRESS_99 * FW_PERCENTAGE_100) &&
                (pogo_keyboard_client->fw_update_progress >= FW_PROGRESS_96 * FW_PERCENTAGE_100));

        if (pogo_keyboard_client->fw_update_progress > FW_PROGRESS_99 * FW_PERCENTAGE_100) {
            kb_info("kb ota not rest,maybe KB plugout\n");
            pogo_keyboard_client->kpd_fw_status = FW_UPDATE_FAIL;
            pogo_keyboard_client->fw_update_progress = FW_PROGERSS_0;
            max_disconnect_count = DEFAULT_DISCONNECT_COUNT;
            return -EIO;
        }
    }

    return ret;
}

static int handle_dfu_ota_update(const unsigned char *firmware_data, u32 fw_data_count)
{
    int ret;

    if (!bat_data || !bin_data || !tp_data) {
        kb_err("bat_data or bin_data or tp_data is NULL\n");
        return -EINVAL;
    }

    ret = kpd_fw_dfu_update(firmware_data, fw_data_count, bat_data->checksum,
                          bat_data->start_addr, bat_data->file_len, bat_data->type);
    if (ret != 0) {
        return ret;
    }

    ret = handle_tp_ota_update(firmware_data, fw_data_count);
    if (ret != 0) {
        return ret;
    }

    ret = handle_kb_ota_update(firmware_data, fw_data_count);
    return ret;
}

static int handle_standard_ota_update(const unsigned char *firmware_data, u32 fw_data_count,
                                     u32 checksum, int version)
{
    return kpd_fw_update(firmware_data, fw_data_count, checksum, version);
}

static void cleanup_resources(const struct firmware *fw_entry)
{
    if (fw_entry) {
        release_firmware(fw_entry);
    }
    cleanup_dfu_data();
}

void kpdmcu_fw_data_version_thread(struct work_struct *work)
{
    const struct firmware *fw_entry = NULL;
    u32 checksum = 0;
    int version = 0;
    int ret = 0;

    if(pogo_keyboard_client == NULL) {
        kb_err("%s %d pogo_keyboard_client is NULL!!!\n", __func__, __LINE__);
        return;
    }

    if(pogo_keyboard_client->ota_firmware_name == NULL) {
        kb_err("%s %d ota_firmware_name is NULL!!!\n", __func__, __LINE__);
        return;
    }

    cleanup_dfu_data();

    fw_entry = get_fw_firmware(pogo_keyboard_client, pogo_keyboard_client->ota_firmware_name);
    if(fw_entry != NULL) {
        pogo_keyboard_client->kpdmcu_fw_cnt = (u32)(fw_entry->size / 1024);
        kb_debug("%s %d, kpdmcu_fw_cnt:%uKB\n", __func__, __LINE__, pogo_keyboard_client->kpdmcu_fw_cnt);
        if (pogo_keyboard_client->pogopin_ota_dfu)
            ret = kpd_fw_dfu_isvalid(fw_entry->data, fw_entry->size);
        else
            ret = kpd_fw_isvalid(fw_entry->data, fw_entry->size, &checksum, &version);
        if (ret) {
            kb_err("%s %d kpd fw is not valid!!!\n", __func__, __LINE__);
            cleanup_dfu_data();
        }
    } else {
        kb_err("%s %d kpd mcu request firmware fail\n", __func__, __LINE__);
        return;
    }

    release_firmware(fw_entry);
    fw_entry = NULL;
}

static int ensure_lcd_state(struct pogo_keyboard_data *client)
{
    int ret = 0;
    if ((client->pogo_keyboard_status & KEYBOARD_LCD_ON_STATUS) == 0) {
        ret = pogo_keyboard_set_lcd_state(true);
        if (ret) {
            kb_err("pogo_keyboard_set_lcd_state err!\n");
            client->kpd_fw_status = FW_UPDATE_FAIL;
            return ret;
        }
    }
    return 0;
}

void kpdmcu_fw_update_thread(struct work_struct *work)
{
    const unsigned char *firmware_data = NULL;
    u32 fw_data_count = 0;
    u32 checksum = 0;
    int version = 0;
    const struct firmware *fw_entry = NULL;
    int ret = 0;

    if (pogo_keyboard_client == NULL) {
        pogo_keyboard_client->kpd_fw_status = FW_UPDATE_READY;
        return;
    }

    if (pogo_keyboard_client->pogopin_wakelock)
        __pm_stay_awake(pogo_keyboard_client->pogopin_wakelock);
    mutex_lock(&pogo_keyboard_client->mutex);

    cleanup_dfu_data();

    pogo_keyboard_client->kpd_fw_status = FW_UPDATE_START;
    pogo_keyboard_client->kpdmcu_update_end = false;
    pogo_keyboard_client->fw_update_progress = FW_PROGERSS_1 * FW_PERCENTAGE_100;

    if (pogo_keyboard_client->is_kpdmcu_need_fw_update == false &&
        pogo_keyboard_client->kpdmcu_fw_update_force == false) {
        kb_info("not need fw update\n");
        goto success;
    }

    ret = handle_firmware_validation(&fw_entry, &fw_data_count, &firmware_data, &checksum, &version);
    if (ret) {
        pogo_keyboard_client->kpd_fw_status = FW_UPDATE_FAIL;
        goto cleanup;
    }

    if (ensure_lcd_state(pogo_keyboard_client) != 0) {
        goto cleanup;
    }

    pogo_keyboard_client->fw_update_progress = FW_PROGRESS_2 * FW_PERCENTAGE_100;

    if (pogo_keyboard_client->pogopin_ota_dfu) {
        ret = handle_dfu_ota_update(firmware_data, fw_data_count);
    } else {
        ret = handle_standard_ota_update(firmware_data, fw_data_count, checksum, version);
    }

    if (ret) {
        pogo_keyboard_client->kpd_fw_status = FW_UPDATE_FAIL;
        goto cleanup;
    }

    pogo_keyboard_client->kpdmcu_update_end = true;

success:
    pogo_keyboard_client->kpd_fw_status = FW_UPDATE_SUC;
    pogo_keyboard_client->fw_update_progress = FW_PROGRESS_100 * FW_PERCENTAGE_100;
    max_disconnect_count = DEFAULT_DISCONNECT_COUNT;
    kb_info("fw update success!\n");

cleanup:
    cleanup_resources(fw_entry);
    mutex_unlock(&pogo_keyboard_client->mutex);
    if (pogo_keyboard_client->pogopin_wakelock)
        __pm_relax(pogo_keyboard_client->pogopin_wakelock);
}
