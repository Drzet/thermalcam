#include "visible_camera.h"
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include "esp_log.h"
#include "esp_video_init.h"
#include "linux/videodev2.h"

static const char *TAG = "visible_csi";

void visible_camera_close(visible_camera_t *cam)
{
    if (!cam) return;
    if (cam->fd >= 0) {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        (void)ioctl(cam->fd, VIDIOC_STREAMOFF, &type);
        for (unsigned i = 0; i < cam->count; ++i) {
            if (cam->buffer[i]) munmap(cam->buffer[i], cam->size[i]);
        }
        close(cam->fd);
    }
    memset(cam, 0, sizeof(*cam));
    cam->fd = -1;
}

esp_err_t visible_camera_init(visible_camera_t *cam)
{
    if (!cam) return ESP_ERR_INVALID_ARG;
    memset(cam, 0, sizeof(*cam));
    cam->fd = -1;

    /* The sensor configuration, CSI controller and ISP are managed by
     * Espressif esp_video; no sensor-register table is duplicated here. */
    esp_video_init_csi_config_t csi = {
        .sccb_config = {
            .init_sccb = true,
            .i2c_config = {
                .port = 0,
                .sda_pin = 7,
                .scl_pin = 8,
            },
            .freq = 400000,
        },
        .reset_pin = -1,
        .pwdn_pin = -1,
    };
    esp_video_init_config_t config = { .csi = &csi };
    esp_err_t err = esp_video_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_video_init: %s", esp_err_to_name(err));
        return err;
    }

    cam->fd = open("/dev/video0", O_RDONLY);
    if (cam->fd < 0) {
        ESP_LOGE(TAG, "Unable to open MIPI CSI /dev/video0");
        return ESP_FAIL;
    }

    struct v4l2_format fmt = {
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .fmt.pix = {
            .width = VISIBLE_W,
            .height = VISIBLE_H,
            .pixelformat = V4L2_PIX_FMT_RGB565,
        },
    };
    if (ioctl(cam->fd, VIDIOC_S_FMT, &fmt) < 0 ||
        fmt.fmt.pix.width != VISIBLE_W ||
        fmt.fmt.pix.height != VISIBLE_H ||
        fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_RGB565) {
        ESP_LOGE(TAG, "OV5647 RGB565 %dx%d format unavailable", VISIBLE_W, VISIBLE_H);
        visible_camera_close(cam);
        return ESP_ERR_NOT_SUPPORTED;
    }

    struct v4l2_requestbuffers req = {
        .count = VISIBLE_BUFFER_COUNT,
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_MMAP,
    };
    if (ioctl(cam->fd, VIDIOC_REQBUFS, &req) < 0 ||
        req.count < VISIBLE_BUFFER_COUNT) {
        ESP_LOGE(TAG, "CSI mmap buffers unavailable");
        visible_camera_close(cam);
        return ESP_FAIL;
    }
    cam->count = VISIBLE_BUFFER_COUNT;
    for (unsigned i = 0; i < cam->count; ++i) {
        struct v4l2_buffer buf = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
            .index = i,
        };
        if (ioctl(cam->fd, VIDIOC_QUERYBUF, &buf) < 0 ||
            buf.length < VISIBLE_W * VISIBLE_H * 2) {
            ESP_LOGE(TAG, "CSI frame buffer %u invalid", i);
            visible_camera_close(cam);
            return ESP_FAIL;
        }
        void *ptr = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED,
                         cam->fd, buf.m.offset);
        if (ptr == MAP_FAILED) {
            ESP_LOGE(TAG, "CSI mmap failed, buffer %u", i);
            visible_camera_close(cam);
            return ESP_ERR_NO_MEM;
        }
        cam->buffer[i] = ptr;
        cam->size[i] = buf.length;
        if (ioctl(cam->fd, VIDIOC_QBUF, &buf) < 0) {
            ESP_LOGE(TAG, "CSI QBUF failed, buffer %u", i);
            visible_camera_close(cam);
            return ESP_FAIL;
        }
    }
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(cam->fd, VIDIOC_STREAMON, &type) < 0) {
        ESP_LOGE(TAG, "CSI STREAMON failed");
        visible_camera_close(cam);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "OV5647 CSI/ISP RGB565 %dx%d streaming", VISIBLE_W, VISIBLE_H);
    return ESP_OK;
}

bool visible_camera_capture(visible_camera_t *cam, const uint16_t **pixels,
                            unsigned *index)
{
    if (!cam || cam->fd < 0 || !pixels || !index) return false;
    struct v4l2_buffer buf = {
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_MMAP,
    };
    if (ioctl(cam->fd, VIDIOC_DQBUF, &buf) < 0) return false;
    if (buf.index >= cam->count) {
        ESP_LOGE(TAG, "Invalid CSI buffer index %lu", (unsigned long)buf.index);
        return false;
    }
    /* Some P4 CSI implementations report the RAW8 payload size in bytesused
     * even when ISP wrote a full RGB565 buffer. QUERYBUF length was already
     * checked against the complete RGB565 footprint; trust the error flag. */
    if (buf.flags & V4L2_BUF_FLAG_ERROR) {
        ESP_LOGW(TAG, "CSI frame flagged as invalid");
        (void)ioctl(cam->fd, VIDIOC_QBUF, &buf);
        return false;
    }
    *pixels = (const uint16_t *)cam->buffer[buf.index];
    *index = buf.index;
    return true;
}

void visible_camera_release(visible_camera_t *cam, unsigned index)
{
    if (!cam || cam->fd < 0 || index >= cam->count) return;
    struct v4l2_buffer buf = {
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_MMAP,
        .index = index,
    };
    if (ioctl(cam->fd, VIDIOC_QBUF, &buf) < 0)
        ESP_LOGE(TAG, "CSI frame %u requeue failed", index);
}
