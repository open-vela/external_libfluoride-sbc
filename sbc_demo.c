#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sbc_encoder.h>
#include <oi_codec_sbc.h>

static void print_usage( char* argv[] )
{
    fprintf(stdout, "Usage: %s [-e] [msbc/sbc] [sampling rate (Hz)] [channels (1/2)] "
            "[bitrate (bps)] [input] [output]\n", argv[0]);
    fprintf(stdout, "       %s [-d] [input] [output]\n\n", argv[0]);
}


typedef struct test_sbc_param {
    FILE *fin;
    FILE *fout;
    uint8_t *in;
    uint8_t *out;
    int frame_size;
    int pkt_size;
    int channels;
    int bitrate; /*bps*/
    int sample_rate; /*HZ*/
    int msbc;
    int encode;
}test_sbc_param;


#define MSBC_PKT_LEN 57
#define SBC_MAX_PKT_LEN (SBC_MAX_CHANNELS * SBC_MAX_BLOCKS * SBC_MAX_BANDS * 4 \
        + SBC_CODEC_MIN_FILTER_BUFFERS * SBC_MAX_BANDS * SBC_MAX_CHANNELS * 2)
#define MSBC_FRAME_SIZE 120
#define SAMPLE_BYTES 2

static void sbc_encode(test_sbc_param* test_param) {
    SBC_ENC_PARAMS param;

    memset(&param, 0, sizeof(SBC_ENC_PARAMS));

    if (test_param->msbc) {
        param.s16SamplingFreq = SBC_sf16000;
        param.s16ChannelMode = SBC_MONO;
        param.s16NumOfSubBands = 8;
        param.s16NumOfChannels = 1;
        param.s16NumOfBlocks = 15;
        param.s16AllocationMethod = SBC_LOUDNESS;
        param.s16BitPool = 26;
        param.Format = SBC_FORMAT_MSBC;
        test_param->frame_size = 120;
    } else {
        if (test_param->channels == 1) {
            param.s16ChannelMode = SBC_MONO;

            if (test_param->bitrate > 270000)
                param.s16NumOfSubBands = 4;
            else
                param.s16NumOfSubBands = 8;
        } else {
            if (test_param->bitrate < 180000 || test_param->bitrate > 420000)
                param.s16ChannelMode = SBC_JOINT_STEREO;
            else
                param.s16ChannelMode = SBC_STEREO;

            if (test_param->bitrate > 420000)
                param.s16NumOfSubBands = 4;
            else
                param.s16NumOfSubBands = 8;
            }

        param.s16NumOfBlocks = SBC_MAX_NUM_OF_BLOCKS;
        param.s16AllocationMethod = SBC_LOUDNESS;
        param.u16BitRate = test_param->bitrate / 1000;
        param.s16NumOfChannels = test_param->channels;

        switch (test_param->sample_rate)
        {
            case 16000:
                param.s16SamplingFreq = SBC_sf16000;
                break;
            case 32000:
                param.s16SamplingFreq = SBC_sf32000;
                break;
            case 44100:
                param.s16SamplingFreq = SBC_sf44100;
                break;
            default:
                param.s16SamplingFreq = SBC_sf48000;
                break;
        }

        test_param->frame_size = 4*((param.s16NumOfSubBands >> 3) + 1) * 4*(param.s16NumOfBlocks >> 2) * test_param->channels;

        SBC_Encoder_Init(&param);
    }


    int joint = param.s16ChannelMode == SBC_JOINT_STEREO;
    int dual  = param.s16ChannelMode == SBC_DUAL;

    test_param->pkt_size = 4 + (4 * param.s16NumOfSubBands * param.s16NumOfChannels) / 8
                     + ((param.s16NumOfBlocks * param.s16BitPool * (1 + dual)
                     + joint * param.s16NumOfSubBands) + 7) / 8;

    test_param->in = (uint8_t*)malloc(test_param->frame_size * SAMPLE_BYTES);
    test_param->out = (uint8_t*)malloc(test_param->pkt_size);
    if (!test_param->in || !test_param->out) {
        printf("%s %d Decode: malloc failed!\n",__func__, __LINE__);
        goto out;
    }

    while(1) {
        memset(test_param->in, 0, test_param->frame_size * SAMPLE_BYTES);
        memset(test_param->out, 0, test_param->pkt_size);

        int num_read = fread(test_param->in, 1, test_param->frame_size * SAMPLE_BYTES, test_param->fin);
        if (num_read < test_param->frame_size * SAMPLE_BYTES) {
            printf("%s %d Encode:fread finish, num_read %d, \n",__func__, __LINE__, num_read);
            goto out;
        }

        int encret = SBC_Encode(&param, (int16_t *)test_param->in, test_param->out);
        if (encret != test_param->pkt_size) {
            printf("%s %d Encode: encode frame error, frame length %d, act  %u \n",
                  __func__, __LINE__, test_param->pkt_size, encret);
            goto out;
        }

        int num_write = fwrite(test_param->out, 1, test_param->pkt_size, test_param->fout);
        if (num_write != test_param->pkt_size) {
            printf("%s %d Decode: write failed!, num_write %d, \n",__func__, __LINE__, num_write);
            goto out;
        }
    }

out:
    if (test_param->in)
        free(test_param->in);
    if (test_param->out)
        free(test_param->out);
    if (test_param->fin)
        fclose(test_param->fin);
    if (test_param->fout)
        fclose(test_param->fout);
}

static void sbc_decode(test_sbc_param* test_param) {
    OI_STATUS status;
    OI_CODEC_SBC_DECODER_CONTEXT context;
    int blocks, mode, subbands, bitpool, channels, joint;
    uint8_t data[SBC_MAX_PKT_LEN + 3];
    uint8_t header[3];

    fread(header, 1, 3, test_param->fin);
    fseek(test_param->fin, 0, SEEK_SET);

    if (header[0] != 0x9c && header[0] != 0xad) {
       printf("%s %d Decode: Invalid input!\n",__func__, __LINE__);
       goto out;
    }

    test_param->msbc = header[0] == 0xad;

    if (test_param->msbc) {
        status = OI_CODEC_SBC_DecoderReset(&context, (uint32_t *)data,
                                           sizeof(data), 1, 1, false);
        if (!OI_SUCCESS(status))
            goto out;

        status = OI_CODEC_SBC_DecoderConfigureMSbc(&context);
        if (!OI_SUCCESS(status))
            goto out;

        test_param->frame_size = MSBC_FRAME_SIZE;
        test_param->pkt_size = MSBC_PKT_LEN;
    } else {
        blocks   = (((header[1] >> 4) & 0x03) + 1) << 2;
        mode     =   (header[1] >> 2) & 0x03;
        subbands = (((header[1] >> 0) & 0x01) + 1) << 2;
        bitpool  = header[2];

        channels = mode == SBC_MONO ? 1 : 2;
        joint    = mode == SBC_JOINT_STEREO;

        status = OI_CODEC_SBC_DecoderReset(&context, (uint32_t *)data,
                                           sizeof(data), 2, channels, false);
        if (!OI_SUCCESS(status))
            goto out;

        test_param->pkt_size = 4 + (subbands * channels) / 2
             + ((((mode == SBC_DUAL) + 1) * blocks * bitpool
                 + (joint * subbands)) + 7) / 8;
        test_param->frame_size = subbands * blocks * channels;
    }

    printf("%s %d Decode: pkt_size:%d frame_size:%d\n",__func__, __LINE__,
           test_param->pkt_size, test_param->frame_size);

    test_param->in = (uint8_t*)malloc(test_param->pkt_size);
    test_param->out = (uint8_t*)malloc(test_param->frame_size * SAMPLE_BYTES);
    if (!test_param->in || !test_param->out) {
        printf("%s %d Decode: malloc failed!\n",__func__, __LINE__);
        goto out;
    }

    while(1) {
        memset(test_param->in, 0, test_param->pkt_size);
        memset(test_param->out, 0, test_param->frame_size * SAMPLE_BYTES);

        int num_read = fread(test_param->in, 1, test_param->pkt_size, test_param->fin);
        if (num_read < test_param->pkt_size) {
            printf("%s %d Decode: fread finish, num_read %d, \n",__func__, __LINE__, num_read);
            goto out;
        }

        const OI_BYTE* indata = (const OI_BYTE*)test_param->in;
        uint32_t in_size = test_param->pkt_size;
        uint32_t out_size = test_param->frame_size * SAMPLE_BYTES;

        status = OI_CODEC_SBC_DecodeFrame(&context, &indata,
                                        &in_size, (int16_t *)test_param->out, &out_size);
        if (!OI_SUCCESS(status)) {
            printf("%s %d Decode: status: %d\n", __func__, __LINE__, status);
            goto out;
        }
        int num_write = fwrite(test_param->out, 1, out_size, test_param->fout);
        if (num_write != out_size) {
            printf("%s %d Decode: write failed!, num_write %d, \n",__func__, __LINE__, num_write);
            goto out;
        }
    }

out:
    if (test_param->in)
        free(test_param->in);
    if (test_param->out)
        free(test_param->out);
    if (test_param->fin)
        fclose(test_param->fin);
    if (test_param->fout)
        fclose(test_param->fout);
}

int main(int argc, char *argv[])
{
    test_sbc_param test_param;

    memset(&test_param, 0, sizeof(test_sbc_param));

    if (argc < 3 )
    {
       print_usage(argv);
       return -1;
    }

    if (strcmp(argv[1], "-e")==0)
    {
        test_param.encode = 1;

        if (argc < 7 )
        {
            print_usage(argv);
            return -1;
        }

        if (strcmp(argv[2], "msbc")==0)
        {
            test_param.msbc = 1;
        } else if (strcmp(argv[2], "sbc")==0)
        {
            test_param.msbc = 0;
        } else {
            print_usage(argv);
            return -1;
        }

        test_param.sample_rate = atol(argv[3]);
        test_param.channels = atol(argv[4]);
        test_param.bitrate = atol(argv[5]);
        test_param.fin = fopen(argv[6], "rb");
        test_param.fout = fopen(argv[7], "wb+");

    } else if (strcmp(argv[1], "-d")==0)
    {
        test_param.encode = 0;
        test_param.fin = fopen(argv[2], "rb");
        test_param.fout = fopen(argv[3], "wb+");
    } else {
        print_usage(argv);
        return -1;
    }

    if (!test_param.fin || !test_param.fout) {
         printf("%s %d open file failed! in:%p out:%p\n",__func__, __LINE__, test_param.fin, test_param.fout);
         return -1;
    }

    if (test_param.encode)
        sbc_encode(&test_param);
    else
        sbc_decode(&test_param);

    return 0;
}
