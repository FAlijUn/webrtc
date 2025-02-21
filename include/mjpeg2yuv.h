#pragma once
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/time.h>
#include <rockchip/rk_mpi.h>
#include <rockchip/rk_type.h>
#include <rga/RgaApi.h>
#include <rga/RockchipRga.h> 

#define MPP_ALIGN(x, a) (((x) + (a)-1) & ~((a)-1))

class MjpegToYuvConverter{
public:
  MjpegToYuvConverter(){
    std::cout << "MjpegToYuvConverter constructor" << std::endl;
  }

  bool init(){
    std::cout << "MjpegToYuvConverter init" << std::endl;
    rgb_buffer_ = new uint8_t[640 * 480 * 3];
    ret_ = mpp_create(&mpp_ctx_, &mpp_api_);
    if (ret_ != MPP_OK) {
      std::cout << "mpp_create failed, ret = " << ret_ << std::endl;
      return false;
    }
    MpiCmd mpi_cmd = MPP_CMD_BASE;
    MppParam mpp_param = nullptr;

    mpi_cmd = MPP_DEC_SET_PARSER_SPLIT_MODE;
    mpp_param = &need_split_;
    ret_ = mpp_api_->control(mpp_ctx_, mpi_cmd, mpp_param);
    if (ret_ != MPP_OK) {
      std::cout << "mpp_api control failed, ret = " << ret_ << std::endl;
      return false;
    }

    ret_ = mpp_init(mpp_ctx_, MPP_CTX_DEC, MPP_VIDEO_CodingMJPEG);
    if (ret_ != MPP_OK) {
      std::cout << "mpp_init failed, ret = " << ret_ << std::endl;
      return false;
    }
    MppFrameFormat fmt = MPP_FMT_YUV420SP;
    mpp_param = &fmt;
    ret_ = mpp_api_->control(mpp_ctx_, MPP_DEC_SET_OUTPUT_FORMAT, mpp_param);
    if (ret_ != MPP_OK) {
      std::cout << "mpp_api control failed, ret = " << ret_ << std::endl;
      return false;
    }

    ret_ = mpp_frame_init(&mpp_frame_);
    if (ret_ != MPP_OK) {
      std::cout << "mpp_frame_init failed, ret = " << ret_ << std::endl;
      return false;
    }

    ret_ = mpp_buffer_group_get_internal(&mpp_frame_group_, MPP_BUFFER_TYPE_ION);
    if (ret_ != MPP_OK) {
      std::cout << "mpp_buffer_group_get_internal failed, ret = " << ret_ << std::endl;
      return false;
    }
    ret_ = mpp_buffer_group_get_internal(&mpp_packet_group_, MPP_BUFFER_TYPE_ION);
    if (ret_ != MPP_OK) {
      std::cout << "mpp_buffer_group_get_internal failed, ret = " << ret_ << std::endl;
      return false;
    }
    RK_U32 hor_stride = MPP_ALIGN(640, 16);
    RK_U32 ver_stride = MPP_ALIGN(480, 16);
    ret_ = mpp_buffer_get(mpp_frame_group_, &mpp_frame_buffer_,
                       hor_stride * ver_stride * 4);
    if (ret_ != MPP_OK) {
      std::cout << "mpp_buffer_get failed, ret = " << ret_ << std::endl;
      return false;
    }

    mpp_frame_set_buffer(mpp_frame_, mpp_frame_buffer_);
    ret_ = mpp_buffer_get(mpp_packet_group_, &mpp_packet_buffer_, 640 * 480 * 3);
    if (ret_ != MPP_OK) {
      std::cout << "mpp_buffer_get failed, ret = " << ret_ << std::endl;
      return false;
    }
    mpp_packet_init_with_buffer(&mpp_packet_, mpp_packet_buffer_);
    data_buffer_ = (uint8_t *)mpp_buffer_get_ptr(mpp_packet_buffer_);
    
    rga_ = new RockchipRga();
    rga_->RkRgaInit();
    return true;
  }

  bool decode(unsigned char* data, uint32_t data_size, uint8_t* yuv_buffer){

    if (!data || data_size == 0) {
      std::cout << "Invalid input data!" << std::endl;
      return false;
    }

   // 准备输入packet
    if (!mpp_packet_) {
      std::cout << "mpp_packet_ is null!" << std::endl;
      return false;
    }

    memset(data_buffer_, 0, 640 * 480 * 3);
    memcpy(data_buffer_, data, data_size);
    
    
    // 准备输入packet
    mpp_packet_set_pos(mpp_packet_, data_buffer_);     // 设置解码包的起始位置
    mpp_packet_set_length(mpp_packet_, data_size);     // 设置解码包的长度
    mpp_packet_set_eos(mpp_packet_);                   // 在处理视频流时，标记解码包是流的结束
    if(mpp_ctx_ == nullptr){
      std::cout << "mpp_ctx_ is null!" << std::endl;
      return false;
    }
    ret_ = mpp_api_->poll(mpp_ctx_, MPP_PORT_INPUT, MPP_POLL_BLOCK); // 等待解码器输入端口准备好
    if(ret_ != MPP_OK){
      std::cout << "mpp_api poll failed" << std::endl;
      return false;
    }
    
    ret_ = mpp_api_->dequeue(mpp_ctx_, MPP_PORT_INPUT, &mpp_task_);  // 从解码器的输入队列中取一个任务
    if(ret_ != MPP_OK){
      std::cout << "mpp_api dequeue failed" << std::endl;
      return false;
    }

    mpp_task_meta_set_packet(mpp_task_, KEY_INPUT_PACKET, mpp_packet_); // 将一个解码任务和一个输入数据包关联
    mpp_task_meta_set_frame(mpp_task_, KEY_OUTPUT_FRAME, mpp_frame_);  // 将一个解码任务和一个输出帧关联
    
    ret_ = mpp_api_->enqueue(mpp_ctx_, MPP_PORT_INPUT, mpp_task_);       // 将任务放入解码器的输入队列
    if(ret_ != MPP_OK){
      std::cout << "mpp_api enqueue failed" << std::endl;
      return false;
    }

    ret_ = mpp_api_->poll(mpp_ctx_, MPP_PORT_OUTPUT, MPP_POLL_BLOCK);   // 等待解码器输出端口准备好
    if(ret_ != MPP_OK){
      std::cout << "mpp_api poll failed" << std::endl;
      return false;
    }

    ret_ = mpp_api_->dequeue(mpp_ctx_, MPP_PORT_OUTPUT, &mpp_task_);    // 从解码器的输出队列中取一个任务
    if(ret_ != MPP_OK){
      std::cout << "mpp_api dequeue failed" << std::endl;
      return false;
    }

    if(mpp_task_){
      MppFrame output_frame = nullptr;
      mpp_task_meta_get_frame(mpp_task_, KEY_OUTPUT_FRAME, &output_frame); // 从任务中获取与输出帧mpp_frame_关联的输出帧,输出帧存储到output_frame变量中
      if(mpp_frame_){
        int width = mpp_frame_get_width(mpp_frame_);
        int height = mpp_frame_get_height(mpp_frame_);
        // std::cout << "width: " << width << " height: " << height << std::endl;
        if (!mppFrame2RGB(mpp_frame_, rgb_buffer_)) {
          std::cout << "mppFrame2RGB failed" << std::endl;
          return false;
        }
        // std::cout << (int) rgb_buffer_[0] << " " << (int) rgb_buffer_[1] << " " << (int) rgb_buffer_[2] << std::endl;

        if(mpp_frame_get_eos(output_frame)){
          std::cout << "mpp_frame_get_eos" << std::endl;
        }
      }
      ret_ = mpp_api_->enqueue(mpp_ctx_, MPP_PORT_OUTPUT, mpp_task_); // 将任务放入解码器的输出队列
      if(ret_ != MPP_OK){
        std::cout << "mpp_api enqueue failed" << std::endl;
        return false;
      }
      return true;
    }
    return true;
  }

  bool mppFrame2RGB(const MppFrame frame, uint8_t *data) {
    int width = mpp_frame_get_width(frame);
    int height = mpp_frame_get_height(frame);
    std::cout << "width: " << width << " height: " << height << std::endl;
    // MppBuffer buffer = mpp_frame_get_buffer(frame);
    // memset(data, 0, width * height * 3);
    // uint8_t* buffer_ptr = (uint8_t *)mpp_buffer_get_ptr(buffer);

    // rga_info_t src_info;
    // rga_info_t dst_info;
    // memset(&src_info, 0, sizeof(rga_info_t));
    // memset(&dst_info, 0, sizeof(rga_info_t));
    // src_info.fd = -1;
    // src_info.mmuFlag = 1;
    // src_info.virAddr = buffer_ptr;
    // src_info.format = RK_FORMAT_YCbCr_420_SP;
    // dst_info.fd = -1;
    // dst_info.mmuFlag = 1;
    // dst_info.virAddr = data;
    // dst_info.format = RK_FORMAT_BGR_888;
    // rga_set_rect(&src_info.rect, 0, 0, width, height, width, height,
    //             RK_FORMAT_YCbCr_420_SP);
    // rga_set_rect(&dst_info.rect, 0, 0, width, height, width, height,
    //             RK_FORMAT_BGR_888);
    // if (rga_->RkRgaBlit(&src_info, &dst_info, NULL) != 0) {
    //   std::cout << "RGA blit failed" << std::endl;
    //   return false;
    // }
    // free(buffer_ptr);
    // free(data);
    return true;
  }

  ~MjpegToYuvConverter(){
    if (mpp_frame_buffer_) {
      mpp_buffer_put(mpp_frame_buffer_);
      mpp_frame_buffer_ = nullptr;
    }
    if (mpp_packet_buffer_) {
      mpp_buffer_put(mpp_packet_buffer_);
      mpp_packet_buffer_ = nullptr;
    }
    if (mpp_frame_group_) {
      mpp_buffer_group_put(mpp_frame_group_);
      mpp_frame_group_ = nullptr;
    }
    if (mpp_packet_group_) {
      mpp_buffer_group_put(mpp_packet_group_);
      mpp_packet_group_ = nullptr;
    }
    if (mpp_frame_) {
      mpp_frame_deinit(&mpp_frame_);
      mpp_frame_ = nullptr;
    }
    if (mpp_packet_) {
      mpp_packet_deinit(&mpp_packet_);
      mpp_packet_ = nullptr;
    }
    if (mpp_ctx_) {
      mpp_destroy(mpp_ctx_);
      mpp_ctx_ = nullptr;
    }
  }
private:
  MPP_RET ret_ = MPP_OK;
  MppCtx mpp_ctx_ = nullptr;
  MppApi* mpp_api_ = nullptr;
  MppPacket mpp_packet_ = nullptr;
  MppFrame mpp_frame_ = nullptr;
  MppDecCfg mpp_dec_cfg_ = nullptr;
  MppBuffer mpp_frame_buffer_ = nullptr;
  MppBuffer mpp_packet_buffer_ = nullptr;
  uint8_t* data_buffer_ = nullptr;
  MppBufferGroup mpp_frame_group_ = nullptr;
  MppBufferGroup mpp_packet_group_ = nullptr;
  MppTask mpp_task_ = nullptr;
  uint32_t need_split_ = 0;
  uint8_t* rgb_buffer_ = nullptr;
  RockchipRga* rga_ = nullptr;
};