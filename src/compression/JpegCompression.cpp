// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <iostream>
#include <cstdint>
#include <cstring>
#include "JpegCompression.h"
#include <stdio.h>
#include "jpeglib.h"
#include <time.h>
#include <ipDeviceCommon/Statistic.h>

JpegCompression::JpegCompression(int t_width, int t_height, rs2_format t_format, int t_bpp)
{
        m_cinfo.err = jpeg_std_error(&m_jerr);
        m_dinfo.err = jpeg_std_error(&m_jerr);
        jpeg_create_compress(&m_cinfo);
        jpeg_create_decompress(&m_dinfo);
        m_format = t_format;
        m_width = t_width;
        m_height = t_height;
        m_bpp = t_bpp;
        m_cinfo.input_components = m_bpp;
        if (m_format == RS2_FORMAT_YUYV || m_format == RS2_FORMAT_UYVY)
        {
                m_cinfo.in_color_space = JCS_YCbCr;
                m_cinfo.input_components = 3; //yuyv and uyvy is 2 bpp, converted to yuv that is 3 bpp.
        }
        else if (m_format == RS2_FORMAT_Y8)
        {
                m_cinfo.in_color_space = JCS_GRAYSCALE;
                m_cinfo.input_components = 1;
        }
        else if (m_format == RS2_FORMAT_RGB8 || m_format == RS2_FORMAT_BGR8)
        {
                m_cinfo.in_color_space = JCS_RGB;
        }
        else
        {
                printf("unsupport format %d on jpeg compression\n", t_format);
        }
        m_rowBuffer = new unsigned char[m_cinfo.input_components * t_width];
        m_destBuffer = (*m_cinfo.mem->alloc_sarray)((j_common_ptr)&m_cinfo, JPOOL_IMAGE, m_cinfo.input_components * t_width, 1);

        jpeg_set_defaults(&m_cinfo);
}

JpegCompression::~JpegCompression()
{
        jpeg_destroy_decompress(&m_dinfo);
        jpeg_destroy_compress(&m_cinfo);
}

void JpegCompression::convertYUYVtoYUV(unsigned char **t_buffer)
{
        for (int i = 0; i < m_cinfo.image_width; i += 2)
        {
                m_rowBuffer[i * 3] = (*t_buffer)[i * 2 + 0];     // Y
                m_rowBuffer[i * 3 + 1] = (*t_buffer)[i * 2 + 1]; // U
                m_rowBuffer[i * 3 + 2] = (*t_buffer)[i * 2 + 3]; // V
                m_rowBuffer[i * 3 + 3] = (*t_buffer)[i * 2 + 2]; // Y
                m_rowBuffer[i * 3 + 4] = (*t_buffer)[i * 2 + 1]; // U
                m_rowBuffer[i * 3 + 5] = (*t_buffer)[i * 2 + 3]; // V
        }
        m_row_pointer[0] = m_rowBuffer;
        (*t_buffer) += m_cinfo.image_width * m_bpp;
}

void JpegCompression::convertUYVYtoYUV(unsigned char **t_buffer)
{
        for (int i = 0; i < m_cinfo.image_width; i += 2)
        {
                m_rowBuffer[i * 3] = (*t_buffer)[i * 2 + 1];     // Y
                m_rowBuffer[i * 3 + 1] = (*t_buffer)[i * 2 + 0]; // U
                m_rowBuffer[i * 3 + 2] = (*t_buffer)[i * 2 + 2]; // V
                m_rowBuffer[i * 3 + 3] = (*t_buffer)[i * 2 + 3]; // Y
                m_rowBuffer[i * 3 + 4] = (*t_buffer)[i * 2 + 0]; // U
                m_rowBuffer[i * 3 + 5] = (*t_buffer)[i * 2 + 2]; // V
        }
        m_row_pointer[0] = m_rowBuffer;
        (*t_buffer) += m_cinfo.image_width * m_bpp;
}

void JpegCompression::convertYUVtoYUYV(unsigned char **t_uncompressBuff)
{
        for (int i = 0; i < m_dinfo.output_width; i += 2)
        {
                (*t_uncompressBuff)[i * 2] = m_destBuffer[0][i * 3];         // Y
                (*t_uncompressBuff)[i * 2 + 1] = m_destBuffer[0][i * 3 + 1]; // U
                (*t_uncompressBuff)[i * 2 + 2] = m_destBuffer[0][i * 3 + 3]; // Y
                (*t_uncompressBuff)[i * 2 + 3] = m_destBuffer[0][i * 3 + 2]; // V
        }
        (*t_uncompressBuff) += m_dinfo.output_width * m_bpp;
}

void JpegCompression::convertYUVtoUYVY(unsigned char **t_uncompressBuff)
{
        for (int i = 0; i < m_dinfo.output_width; i += 2)
        {
                (*t_uncompressBuff)[i * 2] = m_destBuffer[0][i * 3 + 1];     // U
                (*t_uncompressBuff)[i * 2 + 1] = m_destBuffer[0][i * 3 + 0]; // Y
                (*t_uncompressBuff)[i * 2 + 2] = m_destBuffer[0][i * 3 + 2]; // V
                (*t_uncompressBuff)[i * 2 + 3] = m_destBuffer[0][i * 3 + 3]; // Y
        }
        (*t_uncompressBuff) += m_dinfo.output_width * m_bpp;
}

void JpegCompression::convertBGRtoRGB(unsigned char **t_buffer)
{
        for (int i = 0; i <  m_cinfo.image_width * m_bpp; i += 3)
        {
                m_rowBuffer[i] = (*t_buffer)[i + 1];     // B
                m_rowBuffer[i + 1] = (*t_buffer)[i];     // G
                m_rowBuffer[i + 2] = (*t_buffer)[i + 2]; // R
        }
        m_row_pointer[0] = m_rowBuffer;
        (*t_buffer) += m_cinfo.image_width * m_bpp;
}

void JpegCompression::convertRGBtoBGR(unsigned char **t_uncompressBuff)
{
        for (int i = 0; i < m_dinfo.output_width * m_bpp; i += 3)
        {
                (*t_uncompressBuff)[i] = m_destBuffer[0][i + 1];     //B
                (*t_uncompressBuff)[i + 1] = m_destBuffer[0][i];     // G
                (*t_uncompressBuff)[i + 2] = m_destBuffer[0][i + 2]; // R
        }
        (*t_uncompressBuff) += m_dinfo.output_width * m_bpp;
}

int JpegCompression::compressBuffer(unsigned char *t_buffer, int t_size, unsigned char *t_compressedBuf)
{
        long unsigned int compressedSize = 0;
        unsigned char *data;
#ifdef STATISTICS
        Statistic::getStatisticStreams()[rs2_stream::RS2_STREAM_COLOR]->m_compressionBegin = std::chrono::system_clock::now();
#endif
        jpeg_mem_dest(&m_cinfo, &data, &compressedSize);
        m_cinfo.image_width = m_width;
        m_cinfo.image_height = m_height;
        uint64_t row_stride = m_cinfo.image_width * m_cinfo.input_components;
        jpeg_start_compress(&m_cinfo, TRUE);
        while (m_cinfo.next_scanline < m_cinfo.image_height)
        {
                if (m_format == RS2_FORMAT_RGB8 || m_format == RS2_FORMAT_Y8)
                {
                        m_row_pointer[0] = &t_buffer[m_cinfo.next_scanline * row_stride];
                }
                else if (m_format == RS2_FORMAT_YUYV)
                {
                        convertYUYVtoYUV(&t_buffer);
                }
                else if (m_format == RS2_FORMAT_UYVY)
                {
                        convertUYVYtoYUV(&t_buffer);
                }
                else if (m_format == RS2_FORMAT_BGR8)
                {
                        convertBGRtoRGB(&t_buffer);
                }
                else
                {
                        printf("unsupport format %d on jpeg compression\n", m_format);
                        return -1;
                }
                jpeg_write_scanlines(&m_cinfo, m_row_pointer, 1);
        }
        jpeg_finish_compress(&m_cinfo);
        int compressWithHeaderSize = compressedSize + sizeof(int);
        if (compressWithHeaderSize > t_size)
        {
                printf("error: compression overflow, destination buffer is smaller than the compressed size\n");
                return -1;
        }
        memcpy(t_compressedBuf, &compressedSize, sizeof(int));
        memcpy(t_compressedBuf + sizeof(int), data, compressedSize);
        if (m_compFrameCounter++ % 50 == 0)
        {
                printf("finish jpeg color compression, size: %lu, compressed size %u, frameNum: %d \n", t_size, compressedSize, m_compFrameCounter);
        }
#ifdef STATISTICS
        StreamStatistic *st = Statistic::getStatisticStreams()[rs2_stream::RS2_STREAM_COLOR];
        st->m_compressionFrameCounter++;
        std::chrono::system_clock::time_point compressionEnd = std::chrono::system_clock::now();
        st->m_compressionTime = compressionEnd - st->m_compressionBegin;
        st->m_avgCompressionTime += st->m_compressionTime.count();
        printf("STATISTICS: streamType: %d, jpeg compress time: %0.2fm, average: %0.2fm, counter: %d\n", rs2_stream::RS2_STREAM_COLOR, st->m_compressionTime * 1000,
               (st->m_avgCompressionTime * 1000) / st->m_compressionFrameCounter, st->m_compressionFrameCounter);
        st->m_decompressedSizeSum = t_size;
        st->m_compressedSizeSum = compressedSize;
        printf("STATISTICS: streamType: %d, jpeg ratio: %0.2fm, counter: %d\n", rs2_stream::RS2_STREAM_COLOR, st->m_decompressedSizeSum / (float)st->m_compressedSizeSum, st->m_compressionFrameCounter);
#endif
        return compressWithHeaderSize;
}

int JpegCompression::decompressBuffer(unsigned char *t_buffer, int t_compressedSize, unsigned char *t_uncompressedBuf)
{
        unsigned char *ptr = t_uncompressedBuf;
        unsigned char *data = t_buffer;
        unsigned int jpegHeader, res;
#ifdef STATISTICS
        Statistic::getStatisticStreams()[rs2_stream::RS2_STREAM_COLOR]->m_decompressionBegin = std::chrono::system_clock::now();
#endif
        jpeg_mem_src(&m_dinfo, data, t_compressedSize);
        memcpy(&jpegHeader, t_buffer, sizeof(unsigned int));
        if (jpegHeader != 0xE0FFD8FF)
        { //check header integrity if = E0FF D8FF - the First 4 bytes jpeg standards.
                printf("Error: not a jpeg frame, skip frame\n");
                return -1;
        }
        res = jpeg_read_header(&m_dinfo, TRUE);
        if (!res)
        {
                printf("Error: jpeg_read_header failed\n");
                return -1;
        }
        if (m_format == RS2_FORMAT_RGB8 || m_format == RS2_FORMAT_BGR8)
        {
                m_dinfo.out_color_space = JCS_RGB;
        }
        else if (m_format == RS2_FORMAT_YUYV || m_format == RS2_FORMAT_UYVY)
        {
                m_dinfo.out_color_space = JCS_YCbCr;
        }
        else if (m_format == RS2_FORMAT_Y8)
        {
                m_dinfo.out_color_space = JCS_GRAYSCALE;
        }
        else
        {
                printf("unsupport format %d on jpeg compression\n", m_format);
                return -1;
        }
        res = jpeg_start_decompress(&m_dinfo);
        if (!res)
        {
                printf("error: jpeg_start_decompress failed \n");
                return -1;
        }
        uint64_t row_stride = m_dinfo.output_width * m_dinfo.output_components;
        while (m_dinfo.output_scanline < m_dinfo.output_height)
        {
                int numLines = jpeg_read_scanlines(&m_dinfo, m_destBuffer, 1); //todo - check the error: JWRN_JPEG_EOF, Premature end of JPEG file
                if (numLines <= 0)
                {
                        printf("error: jpeg_read_scanlines failed, numline: %d\n", numLines);
                        return -1;
                }
                if (m_format == RS2_FORMAT_RGB8 || m_format == RS2_FORMAT_Y8)
                {
                        memcpy(ptr, m_destBuffer[0], row_stride);
                        ptr += row_stride;
                }
                else if (m_format == RS2_FORMAT_YUYV)
                {
                        convertYUVtoYUYV(&ptr);
                }
                else if (m_format == RS2_FORMAT_UYVY)
                {
                        convertYUVtoUYVY(&ptr);
                }
                else if (m_format == RS2_FORMAT_BGR8)
                {
                        convertRGBtoBGR(&ptr);
                }
        }
        res = jpeg_finish_decompress(&m_dinfo); //todo - check the error: m_jerr_UNKNOWN_MARKER, Unsupported marker type 0x%02x
        if (!res)
        {
                printf("error: jpeg_finish_decompress failed \n");
                return -1;
        }
        int uncompressedSize = m_dinfo.output_width * m_dinfo.output_height * m_bpp;
        if (m_decompFrameCounter++ % 50 == 0)
        {
                printf("finish jpeg color decompression, size: %lu, compressed size %u, frameNum: %d \n", uncompressedSize, t_compressedSize, m_decompFrameCounter);
        }

#ifdef STATISTICS
        StreamStatistic *st = Statistic::getStatisticStreams()[rs2_stream::RS2_STREAM_COLOR];
        st->m_decompressionFrameCounter++;
        std::chrono::system_clock::time_point decompressionEnd = std::chrono::system_clock::now();
        st->m_decompressionTime = decompressionEnd - st->m_decompressionBegin;
        st->m_avgDecompressionTime += st->m_decompressionTime.count();
        printf("STATISTICS: streamType: %d, jpeg decompress time: %0.2fm, average: %0.2fm, counter: %d\n", rs2_stream::RS2_STREAM_COLOR, st->m_decompressionTime * 1000,
               (st->m_avgDecompressionTime * 1000) / st->m_decompressionFrameCounter, st->m_decompressionFrameCounter);
#endif
        return uncompressedSize;
}