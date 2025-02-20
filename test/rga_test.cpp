#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <rga/RockchipRga.h>  // 或直接使用 rga.h

int main() {
    // 初始化 RGA 上下文
    RockchipRga rga;
    rga.RkRgaInit();

    // 输入图像参数（假设为 NV12 格式）
    int src_width = 640;
    int src_height = 480;
    int src_format = RK_FORMAT_YCbCr_420_SP;  // 等同于 RK_FORMAT_YCrCb_420_SP

    // 输出图像参数（BGR888）
    int dst_width = src_width;
    int dst_height = src_height;
    int dst_format = RK_FORMAT_BGR_888;

    // 分配输入/输出缓冲区（示例使用虚拟地址，实际需替换为真实数据）
    char *src_buf;
    int ret = posix_memalign((void**)&src_buf, 16, src_width * src_height * 30 / 2);
    if (ret != 0) {
       printf("Memory allocation failed!\n");
       return -1;
    }
    char *dst_buf = (char *)malloc(dst_width * dst_height * 30);     // BGR888 大小为 w*h*3

    // char *dst_buf;
    //  ret = posix_memalign((void**)&dst_buf, 16, dst_width * dst_height * 3); 
    // if (ret != 0) {
    //   printf("Memory allocation failed!\n");
    //   return -1;
    // }

    // 配置输入图像信息
    rga_info_t src_info;
    memset(&src_info, 0, sizeof(rga_info_t));
    src_info.fd = -1;  // 使用虚拟地址（非 DMA-BUF）
    src_info.virAddr = src_buf;  // 输入数据虚拟地址
    src_info.mmuFlag = 0;        // 启用 MMU

    // 设置输入图像格式和尺寸
    rga_set_rect(&src_info.rect,
        0, 0,                // 左上角坐标
        src_width, src_height, // 图像宽高
        src_width, 0, // 缓冲区 stride（通常等于宽）
        src_format);

    // 配置输出图像信息
    rga_info_t dst_info;
    memset(&dst_info, 0, sizeof(rga_info_t));
    dst_info.fd = -1;
    dst_info.virAddr = dst_buf;  // 输出数据虚拟地址
    dst_info.mmuFlag = 0;

    rga_set_rect(&dst_info.rect,
        0, 0,
        dst_width, dst_height,
        dst_width * 3,          // BGR888 的 stride 为 width*3
        0,
        dst_format);

    // 执行颜色空间转换（YUV420SP -> BGR888）
    ret = rga.RkRgaBlit(&src_info, &dst_info, NULL);
    if (ret != 0) {
        printf("RGA blit failed! Error code: %d\n", ret);
        return -1;
    }
    std::cout << "success"<<std::endl;

    // 释放资源
    free(src_buf);
    free(dst_buf);
    return 0;
}

