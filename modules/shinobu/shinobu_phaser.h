#ifndef SHINOBU_PHASER_H
#define SHINOBU_PHASER_H
#include "miniaudio/miniaudio.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float a;
    float h;
} ma_allpass_delay;

typedef struct {
    ma_node_config nodeConfig;
    ma_uint32 sampleRate;
    float range_min;
    float range_max;
    float rate;
    float feedback;
    float depth;
} ma_phaser_node_config;

typedef struct {
    ma_node_base baseNode;
    float phase;
    float left_h;
    float right_h;
    ma_allpass_delay allpass[2][6];  // [channel][stage]
    float range_min;
    float range_max;
    float rate;
    float feedback;
    float depth;
    float sampleRate;
} ma_phaser_node;

MA_API ma_phaser_node_config ma_phaser_node_config_init(ma_uint32 sampleRate);
MA_API ma_result ma_phaser_node_init(ma_node_graph* pNodeGraph, const ma_phaser_node_config* pConfig, const ma_allocation_callbacks* pAllocationCallbacks, ma_phaser_node* pPhaserNode);
MA_API void ma_phaser_node_uninit(ma_phaser_node* pPhaserNode, const ma_allocation_callbacks* pAllocationCallbacks);

MA_API void ma_phaser_node_set_range_min_hz(ma_phaser_node* pPhaserNode, float min_hz);
MA_API void ma_phaser_node_set_range_max_hz(ma_phaser_node* pPhaserNode, float max_hz);
MA_API void ma_phaser_node_set_rate_hz(ma_phaser_node* pPhaserNode, float rate_hz);
MA_API void ma_phaser_node_set_feedback(ma_phaser_node* pPhaserNode, float feedback);
MA_API void ma_phaser_node_set_depth(ma_phaser_node* pPhaserNode, float depth);

MA_API float ma_phaser_node_get_range_min_hz(ma_phaser_node* pPhaserNode);
MA_API float ma_phaser_node_get_range_max_hz(ma_phaser_node* pPhaserNode);
MA_API float ma_phaser_node_get_rate_hz(ma_phaser_node* pPhaserNode);
MA_API float ma_phaser_node_get_feedback(ma_phaser_node* pPhaserNode);
MA_API float ma_phaser_node_get_depth(ma_phaser_node* pPhaserNode);

#ifdef __cplusplus
}
#endif
#endif

#if defined(MINIAUDIO_IMPLEMENTATION) || defined(MA_IMPLEMENTATION)

static void ma_allpass_delay_init(ma_allpass_delay* delay) {
    delay->a = 0;
    delay->h = 0;
}

static void ma_allpass_delay_set_delay(ma_allpass_delay* delay, float d) {
    delay->a = (1.0f - d) / (1.0f + d);
}

static float ma_allpass_delay_update(ma_allpass_delay* delay, float s) {
    float y = s * -delay->a + delay->h;
    delay->h = y * delay->a + s;
    return y;
}

static void ma_phaser_node_process_pcm_frames(ma_node* pNode, const float** ppFramesIn, ma_uint32* pFrameCountIn, float** ppFramesOut, ma_uint32* pFrameCountOut) {
    ma_phaser_node* pPhaser = (ma_phaser_node*)pNode;

    const float* in_l = ppFramesIn[0];
    const float* in_r = in_l + 1;
    float* out_l = ppFramesOut[0];
    float* out_r = out_l + 1;

    float dmin = pPhaser->range_min / (pPhaser->sampleRate / 2.0f);
    float dmax = pPhaser->range_max / (pPhaser->sampleRate / 2.0f);
    float increment = MA_TAU * (pPhaser->rate / pPhaser->sampleRate);

    for (ma_uint32 i = 0; i < pFrameCountIn[0]; i++) {
        pPhaser->phase += increment;
        while (pPhaser->phase >= MA_TAU) {
            pPhaser->phase -= MA_TAU;
        }

        float d = dmin + (dmax - dmin) * ((sinf(pPhaser->phase) + 1.0f) / 2.0f);

        // Update filter coefficients
        for (int j = 0; j < 6; j++) {
            ma_allpass_delay_set_delay(&pPhaser->allpass[0][j], d);
            ma_allpass_delay_set_delay(&pPhaser->allpass[1][j], d);
        }

        // Process left channel
        float y = in_l[i*2] + pPhaser->left_h * pPhaser->feedback;
        for (int j = 0; j < 6; j++) {
            y = ma_allpass_delay_update(&pPhaser->allpass[0][j], y);
        }
        pPhaser->left_h = y;
        out_l[i*2] = in_l[i*2] + y * pPhaser->depth;

        // Process right channel
        y = in_r[i*2] + pPhaser->right_h * pPhaser->feedback;
        for (int j = 0; j < 6; j++) {
            y = ma_allpass_delay_update(&pPhaser->allpass[1][j], y);
        }
        pPhaser->right_h = y;
        out_r[i*2] = in_r[i*2] + y * pPhaser->depth;
    }

    *pFrameCountOut = *pFrameCountIn;
}

static ma_node_vtable g_ma_phaser_node_vtable = {
    ma_phaser_node_process_pcm_frames,
    NULL,
    1,  // 1 input bus
    1,  // 1 output bus
    MA_NODE_FLAG_CONTINUOUS_PROCESSING
};

MA_API ma_phaser_node_config ma_phaser_node_config_init(ma_uint32 sampleRate) {
    ma_phaser_node_config config;
    MA_ZERO_OBJECT(&config);

    config.nodeConfig = ma_node_config_init();
    config.sampleRate = sampleRate;
    config.range_min = 440.0f;
    config.range_max = 1600.0f;
    config.rate = 0.5f;
    config.feedback = 0.7f;
    config.depth = 1.0f;

    return config;
}

MA_API ma_result ma_phaser_node_init(ma_node_graph* pNodeGraph, const ma_phaser_node_config* pConfig, const ma_allocation_callbacks* pAllocationCallbacks, ma_phaser_node* pPhaserNode) {
    ma_node_config baseConfig = pConfig->nodeConfig;
    baseConfig.vtable = &g_ma_phaser_node_vtable;

    ma_uint32 inputChannels[1] = { 2 };
    ma_uint32 outputChannels[1] = { 2 };
    baseConfig.pInputChannels = inputChannels;
    baseConfig.pOutputChannels = outputChannels;

    ma_result result = ma_node_init(pNodeGraph, &baseConfig, pAllocationCallbacks, &pPhaserNode->baseNode);
    if (result != MA_SUCCESS) {
        return result;
    }

    pPhaserNode->phase = 0;
    pPhaserNode->left_h = 0;
    pPhaserNode->right_h = 0;
    pPhaserNode->sampleRate = pConfig->sampleRate;
    pPhaserNode->range_min = pConfig->range_min;
    pPhaserNode->range_max = pConfig->range_max;
    pPhaserNode->rate = pConfig->rate;
    pPhaserNode->feedback = pConfig->feedback;
    pPhaserNode->depth = pConfig->depth;

    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 6; j++) {
            ma_allpass_delay_init(&pPhaserNode->allpass[i][j]);
        }
    }

    return MA_SUCCESS;
}

MA_API void ma_phaser_node_uninit(ma_phaser_node* pPhaserNode, const ma_allocation_callbacks* pAllocationCallbacks) {
    ma_node_uninit(&pPhaserNode->baseNode, pAllocationCallbacks);
}

// Getter/setter implementations with atomic operations for thread safety
MA_API void ma_phaser_node_set_range_min_hz(ma_phaser_node* pPhaserNode, float min_hz) {
    ma_atomic_exchange_f32(&pPhaserNode->range_min, min_hz);
}

MA_API void ma_phaser_node_set_range_max_hz(ma_phaser_node* pPhaserNode, float max_hz) {
    ma_atomic_exchange_f32(&pPhaserNode->range_max, max_hz);
}

MA_API void ma_phaser_node_set_rate_hz(ma_phaser_node* pPhaserNode, float rate_hz) {
    ma_atomic_exchange_f32(&pPhaserNode->rate, rate_hz);
}

MA_API void ma_phaser_node_set_feedback(ma_phaser_node* pPhaserNode, float feedback) {
    ma_atomic_exchange_f32(&pPhaserNode->feedback, feedback);
}

MA_API void ma_phaser_node_set_depth(ma_phaser_node* pPhaserNode, float depth) {
    ma_atomic_exchange_f32(&pPhaserNode->depth, depth);
}

MA_API float ma_phaser_node_get_range_min_hz(ma_phaser_node* pPhaserNode) {
    return ma_atomic_load_f32(&pPhaserNode->range_min);
}

MA_API float ma_phaser_node_get_range_max_hz(ma_phaser_node* pPhaserNode) {
    return ma_atomic_load_f32(&pPhaserNode->range_max);
}

MA_API float ma_phaser_node_get_rate_hz(ma_phaser_node* pPhaserNode) {
    return ma_atomic_load_f32(&pPhaserNode->rate);
}

MA_API float ma_phaser_node_get_feedback(ma_phaser_node* pPhaserNode) {
    return ma_atomic_load_f32(&pPhaserNode->feedback);
}

MA_API float ma_phaser_node_get_depth(ma_phaser_node* pPhaserNode) {
    return ma_atomic_load_f32(&pPhaserNode->depth);
}

#endif // MINIAUDIO_IMPLEMENTATION
