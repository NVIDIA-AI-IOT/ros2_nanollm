#include <tvm/runtime/packed_func.h>
#include <dlpack/dlpack.h>
#include <cuda_fp16.h>
#include <cutlass/cutlass.h>
#include <cutlass/coord.h>
#include <cutlass/tensor_ref.h>
#include <cutlass/util/host_tensor.h>
#include <tvm/runtime/registry.h>
#include <cutlass/epilogue/thread/linear_combination.h>
#include <cutlass_kernels/fpA_intB_gemm.h>
#include <tvm/runtime/registry.h>
#include <cutlass/epilogue/thread/linear_combination.h>
#include <flash.h>
#include <tvm/runtime/registry.h>
#include <cutlass/epilogue/thread/linear_combination.h>
#include <cutlass_kernels/fpA_intB_gemm.h>
#include <tvm/runtime/registry.h>
#include <cutlass/epilogue/thread/linear_combination.h>
#include <cutlass_kernels/fpA_intB_gemm.h>
#include <tvm/runtime/registry.h>
#include <cutlass/epilogue/thread/linear_combination.h>
#include <flash.h>
#include <tvm/runtime/registry.h>
#include <cutlass/epilogue/thread/linear_combination.h>
#include <cutlass_kernels/fpA_intB_gemm.h>
#include <tvm/runtime/registry.h>
#include <cutlass/epilogue/thread/linear_combination.h>
#include <cutlass_kernels/fpA_intB_gemm.h>
#include <tvm/runtime/registry.h>
#include <cutlass/epilogue/thread/linear_combination.h>
#include <cutlass_kernels/fpA_intB_gemm.h>
#include <tvm/runtime/registry.h>
#include <cutlass/epilogue/thread/linear_combination.h>
#include <cutlass/util/device_rmsnorm.h>
#include <cutlass/layout/matrix.h>
#include <tvm/runtime/registry.h>
#include <cutlass/epilogue/thread/linear_combination.h>
#include <cutlass_kernels/fpA_intB_gemm.h>
#include <tvm/runtime/registry.h>
#include <cutlass/epilogue/thread/linear_combination.h>
#include <cutlass/util/device_rmsnorm.h>
#include <cutlass/layout/matrix.h>
#include <tvm/runtime/registry.h>
#include <cutlass/epilogue/thread/linear_combination.h>
#include <cutlass_kernels/fpA_intB_gemm.h>


void fused_decode8_relax_matmul1_cutlass_(DLTensor* model_layers_0_mlp_gate_up_proj_weight_int8_01, DLTensor* model_layers_0_mlp_gate_up_proj_weight_float16_11, DLTensor* lv66, DLTensor* out0){

  
  
  using namespace fastertransformer;
  constexpr auto QuantOp = cutlass::WeightOnlyQuantOp::PER_COLUMN_SCALE_ONLY;

  int m = lv66->shape[0] * lv66->shape[1];
  int n = model_layers_0_mlp_gate_up_proj_weight_int8_01->shape[1] * 2;
  int k = model_layers_0_mlp_gate_up_proj_weight_int8_01->shape[0];

  auto func = tvm::runtime::Registry::Get("runtime.get_cuda_stream");
  ICHECK(func != nullptr);
  cudaStream_t stream = static_cast<cudaStream_t>((*func)().operator void*());
    
  gemm_fp16_int_bias_act<cutlass::uint4b_t, QuantOp>(static_cast<cutlass::half_t*>(lv66->data),
                static_cast<cutlass::uint4b_t*>(model_layers_0_mlp_gate_up_proj_weight_int8_01->data),
                static_cast<cutlass::half_t*>(model_layers_0_mlp_gate_up_proj_weight_float16_11->data),
                nullptr,
                static_cast<cutlass::half_t*>(out0->data),
                "identity",
                m, n, k, k, 0, nullptr, 0, stream);

}

int fused_decode8_relax_matmul1_cutlass_wrapper_(DLTensor* arg0,
	DLTensor* arg1,
	DLTensor* arg2,
	DLTensor* out0) {
  fused_decode8_relax_matmul1_cutlass_(arg0,
  arg1,
  arg2,
  out0);
  return 0;
}

#ifdef __cplusplus
extern "C" {
#endif
TVM_DLL int32_t fused_decode8_relax_matmul1_cutlass(TVMValue* args, int* type_code, int num_args, TVMValue* out_value, int* out_type_code) {
  DLTensor* arg0 = (DLTensor*)(((TVMValue*)args)[0].v_handle);
  DLTensor* arg1 = (DLTensor*)(((TVMValue*)args)[1].v_handle);
  DLTensor* arg2 = (DLTensor*)(((TVMValue*)args)[2].v_handle);
  DLTensor* ret3 = (DLTensor*)(((TVMValue*)args)[3].v_handle);
  fused_decode8_relax_matmul1_cutlass_wrapper_(arg0,arg1,arg2,ret3);
  return 0;
}
#ifdef __cplusplus
}
#endif

void fused_relax_nn_attention1_cutlass1_(DLTensor* lv16, DLTensor* lv26, DLTensor* lv27, DLTensor* workspace, DLTensor* out0){

  
    int q_head_stride = 128;
    int k_head_stride = 128;
    int v_head_stride = 128;
    int o_head_stride = 128;
    int q_row_stride = q_head_stride * 32;
    int k_row_stride = k_head_stride * 8;
    int v_row_stride = v_head_stride * 8;
    int o_row_stride = o_head_stride * 32;
    int q_batch_stride = q_row_stride * lv16->shape[1];
    int k_batch_stride = k_row_stride * lv26->shape[1];
    int v_batch_stride = v_row_stride * lv26->shape[1];
    int o_batch_stride = o_row_stride * lv16->shape[1];

    auto func = tvm::runtime::Registry::Get("runtime.get_cuda_stream");
    ICHECK(func != nullptr);
    cudaStream_t stream = static_cast<cudaStream_t>((*func)().operator void*());

    flash_attn::flash_attention_forward(
                            static_cast<const cutlass::half_t*>(lv16->data),
    			    static_cast<const cutlass::half_t*>(lv26->data),
    			    static_cast<const cutlass::half_t*>(lv27->data),
    			    static_cast<cutlass::half_t*>(out0->data),
    			    1,
    			    lv16->shape[1],
    			    lv26->shape[1],
    			    32,
    			    8,
    			    128,
    			    q_batch_stride,
    			    k_batch_stride,
    			    v_batch_stride,
    			    o_batch_stride,
    			    q_head_stride,
    			    k_head_stride,
    			    v_head_stride,
    			    o_head_stride,
    			    q_row_stride,
    			    k_row_stride,
    			    v_row_stride,
    			    o_row_stride,
    			    0.08838834764831843,
    			    1,
                            lv26->shape[1],
                            0,
    			    stream);
    
}

int fused_relax_nn_attention1_cutlass1_wrapper_(DLTensor* arg0,
	DLTensor* arg1,
	DLTensor* arg2,
	DLTensor* arg3,
	DLTensor* out0) {
  fused_relax_nn_attention1_cutlass1_(arg0,
  arg1,
  arg2,
  arg3,
  out0);
  return 0;
}

#ifdef __cplusplus
extern "C" {
#endif
TVM_DLL int32_t fused_relax_nn_attention1_cutlass1(TVMValue* args, int* type_code, int num_args, TVMValue* out_value, int* out_type_code) {
  DLTensor* arg0 = (DLTensor*)(((TVMValue*)args)[0].v_handle);
  DLTensor* arg1 = (DLTensor*)(((TVMValue*)args)[1].v_handle);
  DLTensor* arg2 = (DLTensor*)(((TVMValue*)args)[2].v_handle);
  DLTensor* arg3 = (DLTensor*)(((TVMValue*)args)[3].v_handle);
  DLTensor* ret4 = (DLTensor*)(((TVMValue*)args)[4].v_handle);
  fused_relax_nn_attention1_cutlass1_wrapper_(arg0,arg1,arg2,arg3,ret4);
  return 0;
}
#ifdef __cplusplus
}
#endif

void fused_decode8_relax_matmul_cutlass_(DLTensor* model_layers_0_mlp_gate_up_proj_weight_int8_0, DLTensor* model_layers_0_mlp_gate_up_proj_weight_float16_1, DLTensor* lv1, DLTensor* out0){

  
  
  using namespace fastertransformer;
  constexpr auto QuantOp = cutlass::WeightOnlyQuantOp::PER_COLUMN_SCALE_ONLY;

  int m = 1;
  int n = model_layers_0_mlp_gate_up_proj_weight_int8_0->shape[1] * 2;
  int k = model_layers_0_mlp_gate_up_proj_weight_int8_0->shape[0];

  auto func = tvm::runtime::Registry::Get("runtime.get_cuda_stream");
  ICHECK(func != nullptr);
  cudaStream_t stream = static_cast<cudaStream_t>((*func)().operator void*());
    
  gemm_fp16_int_bias_act<cutlass::uint4b_t, QuantOp>(static_cast<cutlass::half_t*>(lv1->data),
                static_cast<cutlass::uint4b_t*>(model_layers_0_mlp_gate_up_proj_weight_int8_0->data),
                static_cast<cutlass::half_t*>(model_layers_0_mlp_gate_up_proj_weight_float16_1->data),
                nullptr,
                static_cast<cutlass::half_t*>(out0->data),
                "identity",
                m, n, k, k, 0, nullptr, 0, stream);

}

int fused_decode8_relax_matmul_cutlass_wrapper_(DLTensor* arg0,
	DLTensor* arg1,
	DLTensor* arg2,
	DLTensor* out0) {
  fused_decode8_relax_matmul_cutlass_(arg0,
  arg1,
  arg2,
  out0);
  return 0;
}

#ifdef __cplusplus
extern "C" {
#endif
TVM_DLL int32_t fused_decode8_relax_matmul_cutlass(TVMValue* args, int* type_code, int num_args, TVMValue* out_value, int* out_type_code) {
  DLTensor* arg0 = (DLTensor*)(((TVMValue*)args)[0].v_handle);
  DLTensor* arg1 = (DLTensor*)(((TVMValue*)args)[1].v_handle);
  DLTensor* arg2 = (DLTensor*)(((TVMValue*)args)[2].v_handle);
  DLTensor* ret3 = (DLTensor*)(((TVMValue*)args)[3].v_handle);
  fused_decode8_relax_matmul_cutlass_wrapper_(arg0,arg1,arg2,ret3);
  return 0;
}
#ifdef __cplusplus
}
#endif

void fused_decode6_relax_matmul_cutlass_(DLTensor* model_layers_0_self_attn_query_key_value_proj_weight_int8_0, DLTensor* model_layers_0_self_attn_query_key_value_proj_weight_float16_1, DLTensor* lv, DLTensor* out0){

  
  
  using namespace fastertransformer;
  constexpr auto QuantOp = cutlass::WeightOnlyQuantOp::PER_COLUMN_SCALE_ONLY;

  int m = 1;
  int n = model_layers_0_self_attn_query_key_value_proj_weight_int8_0->shape[1] * 2;
  int k = model_layers_0_self_attn_query_key_value_proj_weight_int8_0->shape[0];

  auto func = tvm::runtime::Registry::Get("runtime.get_cuda_stream");
  ICHECK(func != nullptr);
  cudaStream_t stream = static_cast<cudaStream_t>((*func)().operator void*());
    
  gemm_fp16_int_bias_act<cutlass::uint4b_t, QuantOp>(static_cast<cutlass::half_t*>(lv->data),
                static_cast<cutlass::uint4b_t*>(model_layers_0_self_attn_query_key_value_proj_weight_int8_0->data),
                static_cast<cutlass::half_t*>(model_layers_0_self_attn_query_key_value_proj_weight_float16_1->data),
                nullptr,
                static_cast<cutlass::half_t*>(out0->data),
                "identity",
                m, n, k, k, 0, nullptr, 0, stream);

}

int fused_decode6_relax_matmul_cutlass_wrapper_(DLTensor* arg0,
	DLTensor* arg1,
	DLTensor* arg2,
	DLTensor* out0) {
  fused_decode6_relax_matmul_cutlass_(arg0,
  arg1,
  arg2,
  out0);
  return 0;
}

#ifdef __cplusplus
extern "C" {
#endif
TVM_DLL int32_t fused_decode6_relax_matmul_cutlass(TVMValue* args, int* type_code, int num_args, TVMValue* out_value, int* out_type_code) {
  DLTensor* arg0 = (DLTensor*)(((TVMValue*)args)[0].v_handle);
  DLTensor* arg1 = (DLTensor*)(((TVMValue*)args)[1].v_handle);
  DLTensor* arg2 = (DLTensor*)(((TVMValue*)args)[2].v_handle);
  DLTensor* ret3 = (DLTensor*)(((TVMValue*)args)[3].v_handle);
  fused_decode6_relax_matmul_cutlass_wrapper_(arg0,arg1,arg2,ret3);
  return 0;
}
#ifdef __cplusplus
}
#endif

void fused_relax_nn_attention_cutlass1_(DLTensor* lv1689, DLTensor* lv1699, DLTensor* lv1700, DLTensor* workspace_2, DLTensor* out0){

  
    int q_head_stride = 128;
    int k_head_stride = 128;
    int v_head_stride = 128;
    int o_head_stride = 128;
    int q_row_stride = q_head_stride * 32;
    int k_row_stride = k_head_stride * 8;
    int v_row_stride = v_head_stride * 8;
    int o_row_stride = o_head_stride * 32;
    int q_batch_stride = q_row_stride * 1;
    int k_batch_stride = k_row_stride * lv1699->shape[1];
    int v_batch_stride = v_row_stride * lv1699->shape[1];
    int o_batch_stride = o_row_stride * 1;

    auto func = tvm::runtime::Registry::Get("runtime.get_cuda_stream");
    ICHECK(func != nullptr);
    cudaStream_t stream = static_cast<cudaStream_t>((*func)().operator void*());

    flash_attn::flash_attention_forward(
                            static_cast<const cutlass::half_t*>(lv1689->data),
    			    static_cast<const cutlass::half_t*>(lv1699->data),
    			    static_cast<const cutlass::half_t*>(lv1700->data),
    			    static_cast<cutlass::half_t*>(out0->data),
    			    1,
    			    1,
    			    lv1699->shape[1],
    			    32,
    			    8,
    			    128,
    			    q_batch_stride,
    			    k_batch_stride,
    			    v_batch_stride,
    			    o_batch_stride,
    			    q_head_stride,
    			    k_head_stride,
    			    v_head_stride,
    			    o_head_stride,
    			    q_row_stride,
    			    k_row_stride,
    			    v_row_stride,
    			    o_row_stride,
    			    0.08838834764831843,
    			    1,
                            lv1699->shape[1],
                            0,
    			    stream);
    
}

int fused_relax_nn_attention_cutlass1_wrapper_(DLTensor* arg0,
	DLTensor* arg1,
	DLTensor* arg2,
	DLTensor* arg3,
	DLTensor* out0) {
  fused_relax_nn_attention_cutlass1_(arg0,
  arg1,
  arg2,
  arg3,
  out0);
  return 0;
}

#ifdef __cplusplus
extern "C" {
#endif
TVM_DLL int32_t fused_relax_nn_attention_cutlass1(TVMValue* args, int* type_code, int num_args, TVMValue* out_value, int* out_type_code) {
  DLTensor* arg0 = (DLTensor*)(((TVMValue*)args)[0].v_handle);
  DLTensor* arg1 = (DLTensor*)(((TVMValue*)args)[1].v_handle);
  DLTensor* arg2 = (DLTensor*)(((TVMValue*)args)[2].v_handle);
  DLTensor* arg3 = (DLTensor*)(((TVMValue*)args)[3].v_handle);
  DLTensor* ret4 = (DLTensor*)(((TVMValue*)args)[4].v_handle);
  fused_relax_nn_attention_cutlass1_wrapper_(arg0,arg1,arg2,arg3,ret4);
  return 0;
}
#ifdef __cplusplus
}
#endif

void fused_decode6_relax_matmul1_cutlass_(DLTensor* model_layers_0_self_attn_query_key_value_proj_weight_int8_01, DLTensor* model_layers_0_self_attn_query_key_value_proj_weight_float16_11, DLTensor* lv65, DLTensor* out0){

  
  
  using namespace fastertransformer;
  constexpr auto QuantOp = cutlass::WeightOnlyQuantOp::PER_COLUMN_SCALE_ONLY;

  int m = lv65->shape[0] * lv65->shape[1];
  int n = model_layers_0_self_attn_query_key_value_proj_weight_int8_01->shape[1] * 2;
  int k = model_layers_0_self_attn_query_key_value_proj_weight_int8_01->shape[0];

  auto func = tvm::runtime::Registry::Get("runtime.get_cuda_stream");
  ICHECK(func != nullptr);
  cudaStream_t stream = static_cast<cudaStream_t>((*func)().operator void*());
    
  gemm_fp16_int_bias_act<cutlass::uint4b_t, QuantOp>(static_cast<cutlass::half_t*>(lv65->data),
                static_cast<cutlass::uint4b_t*>(model_layers_0_self_attn_query_key_value_proj_weight_int8_01->data),
                static_cast<cutlass::half_t*>(model_layers_0_self_attn_query_key_value_proj_weight_float16_11->data),
                nullptr,
                static_cast<cutlass::half_t*>(out0->data),
                "identity",
                m, n, k, k, 0, nullptr, 0, stream);

}

int fused_decode6_relax_matmul1_cutlass_wrapper_(DLTensor* arg0,
	DLTensor* arg1,
	DLTensor* arg2,
	DLTensor* out0) {
  fused_decode6_relax_matmul1_cutlass_(arg0,
  arg1,
  arg2,
  out0);
  return 0;
}

#ifdef __cplusplus
extern "C" {
#endif
TVM_DLL int32_t fused_decode6_relax_matmul1_cutlass(TVMValue* args, int* type_code, int num_args, TVMValue* out_value, int* out_type_code) {
  DLTensor* arg0 = (DLTensor*)(((TVMValue*)args)[0].v_handle);
  DLTensor* arg1 = (DLTensor*)(((TVMValue*)args)[1].v_handle);
  DLTensor* arg2 = (DLTensor*)(((TVMValue*)args)[2].v_handle);
  DLTensor* ret3 = (DLTensor*)(((TVMValue*)args)[3].v_handle);
  fused_decode6_relax_matmul1_cutlass_wrapper_(arg0,arg1,arg2,ret3);
  return 0;
}
#ifdef __cplusplus
}
#endif

void fused_decode9_relax_matmul_relax_add1_cutlass_(DLTensor* model_layers_0_mlp_down_proj_weight_int8_01, DLTensor* model_layers_0_mlp_down_proj_weight_float16_11, DLTensor* lv54, DLTensor* lv46, DLTensor* out0){

  
  
  using namespace fastertransformer;
  constexpr auto QuantOp = cutlass::WeightOnlyQuantOp::PER_COLUMN_SCALE_ONLY;

  int m = lv54->shape[0] * lv54->shape[1];
  int n = model_layers_0_mlp_down_proj_weight_int8_01->shape[1] * 2;
  int k = model_layers_0_mlp_down_proj_weight_int8_01->shape[0];

  auto func = tvm::runtime::Registry::Get("runtime.get_cuda_stream");
  ICHECK(func != nullptr);
  cudaStream_t stream = static_cast<cudaStream_t>((*func)().operator void*());
    
  gemm_fp16_int_bias_act<cutlass::uint4b_t, QuantOp>(static_cast<cutlass::half_t*>(lv54->data),
                static_cast<cutlass::uint4b_t*>(model_layers_0_mlp_down_proj_weight_int8_01->data),
                static_cast<cutlass::half_t*>(model_layers_0_mlp_down_proj_weight_float16_11->data),
                static_cast<cutlass::half_t*>(lv46->data),
                static_cast<cutlass::half_t*>(out0->data),
                "identity",
                m, n, k, k, 4096, nullptr, 0, stream);

}

int fused_decode9_relax_matmul_relax_add1_cutlass_wrapper_(DLTensor* arg0,
	DLTensor* arg1,
	DLTensor* arg2,
	DLTensor* arg3,
	DLTensor* out0) {
  fused_decode9_relax_matmul_relax_add1_cutlass_(arg0,
  arg1,
  arg2,
  arg3,
  out0);
  return 0;
}

#ifdef __cplusplus
extern "C" {
#endif
TVM_DLL int32_t fused_decode9_relax_matmul_relax_add1_cutlass(TVMValue* args, int* type_code, int num_args, TVMValue* out_value, int* out_type_code) {
  DLTensor* arg0 = (DLTensor*)(((TVMValue*)args)[0].v_handle);
  DLTensor* arg1 = (DLTensor*)(((TVMValue*)args)[1].v_handle);
  DLTensor* arg2 = (DLTensor*)(((TVMValue*)args)[2].v_handle);
  DLTensor* arg3 = (DLTensor*)(((TVMValue*)args)[3].v_handle);
  DLTensor* ret4 = (DLTensor*)(((TVMValue*)args)[4].v_handle);
  fused_decode9_relax_matmul_relax_add1_cutlass_wrapper_(arg0,arg1,arg2,arg3,ret4);
  return 0;
}
#ifdef __cplusplus
}
#endif

void fused_decode7_relax_matmul_relax_add_cutlass_(DLTensor* model_layers_0_self_attn_o_proj_weight_int8_0, DLTensor* model_layers_0_self_attn_o_proj_weight_float16_1, DLTensor* lv1716, DLTensor* lv1677, DLTensor* out0){

  
  
  using namespace fastertransformer;
  constexpr auto QuantOp = cutlass::WeightOnlyQuantOp::PER_COLUMN_SCALE_ONLY;

  int m = 1;
  int n = model_layers_0_self_attn_o_proj_weight_int8_0->shape[1] * 2;
  int k = model_layers_0_self_attn_o_proj_weight_int8_0->shape[0];

  auto func = tvm::runtime::Registry::Get("runtime.get_cuda_stream");
  ICHECK(func != nullptr);
  cudaStream_t stream = static_cast<cudaStream_t>((*func)().operator void*());
    
  gemm_fp16_int_bias_act<cutlass::uint4b_t, QuantOp>(static_cast<cutlass::half_t*>(lv1716->data),
                static_cast<cutlass::uint4b_t*>(model_layers_0_self_attn_o_proj_weight_int8_0->data),
                static_cast<cutlass::half_t*>(model_layers_0_self_attn_o_proj_weight_float16_1->data),
                static_cast<cutlass::half_t*>(lv1677->data),
                static_cast<cutlass::half_t*>(out0->data),
                "identity",
                m, n, k, k, 0, nullptr, 0, stream);

}

int fused_decode7_relax_matmul_relax_add_cutlass_wrapper_(DLTensor* arg0,
	DLTensor* arg1,
	DLTensor* arg2,
	DLTensor* arg3,
	DLTensor* out0) {
  fused_decode7_relax_matmul_relax_add_cutlass_(arg0,
  arg1,
  arg2,
  arg3,
  out0);
  return 0;
}

#ifdef __cplusplus
extern "C" {
#endif
TVM_DLL int32_t fused_decode7_relax_matmul_relax_add_cutlass(TVMValue* args, int* type_code, int num_args, TVMValue* out_value, int* out_type_code) {
  DLTensor* arg0 = (DLTensor*)(((TVMValue*)args)[0].v_handle);
  DLTensor* arg1 = (DLTensor*)(((TVMValue*)args)[1].v_handle);
  DLTensor* arg2 = (DLTensor*)(((TVMValue*)args)[2].v_handle);
  DLTensor* arg3 = (DLTensor*)(((TVMValue*)args)[3].v_handle);
  DLTensor* ret4 = (DLTensor*)(((TVMValue*)args)[4].v_handle);
  fused_decode7_relax_matmul_relax_add_cutlass_wrapper_(arg0,arg1,arg2,arg3,ret4);
  return 0;
}
#ifdef __cplusplus
}
#endif

void fused_rms_norm_cutlass_(DLTensor* inputs_embeds, DLTensor* model_layers_0_input_layernorm_weight1, DLTensor* out0){

  
    using data_type = cutlass::half_t;
    using namespace cutlass::layout;

    int M = inputs_embeds->shape[0] * inputs_embeds->shape[1];
    int N = 4096;
    cutlass::MatrixCoord size(M, N);
    auto layout_2D = RowMajor::packed(size);
    auto layout_channels = RowMajor::packed({1, N});

    cutlass::TensorRef<data_type, RowMajor> _input((data_type*)inputs_embeds->data, layout_2D);
    cutlass::TensorRef<data_type, RowMajor> _weight((data_type*)model_layers_0_input_layernorm_weight1->data, layout_channels);
    cutlass::TensorRef<data_type, RowMajor> _output((data_type*)out0->data, layout_2D);

    auto func = tvm::runtime::Registry::Get("runtime.get_cuda_stream");
    ICHECK(func != nullptr);
    cudaStream_t stream = static_cast<cudaStream_t>((*func)().operator void*());

    cutlass::rmsnorm(size, _output, _input, _weight, stream, 1e-05);
    
}

int fused_rms_norm_cutlass_wrapper_(DLTensor* arg0,
	DLTensor* arg1,
	DLTensor* out0) {
  fused_rms_norm_cutlass_(arg0,
  arg1,
  out0);
  return 0;
}

#ifdef __cplusplus
extern "C" {
#endif
TVM_DLL int32_t fused_rms_norm_cutlass(TVMValue* args, int* type_code, int num_args, TVMValue* out_value, int* out_type_code) {
  DLTensor* arg0 = (DLTensor*)(((TVMValue*)args)[0].v_handle);
  DLTensor* arg1 = (DLTensor*)(((TVMValue*)args)[1].v_handle);
  DLTensor* ret2 = (DLTensor*)(((TVMValue*)args)[2].v_handle);
  fused_rms_norm_cutlass_wrapper_(arg0,arg1,ret2);
  return 0;
}
#ifdef __cplusplus
}
#endif

void fused_decode7_relax_matmul_relax_add1_cutlass_(DLTensor* model_layers_0_self_attn_o_proj_weight_int8_01, DLTensor* model_layers_0_self_attn_o_proj_weight_float16_11, DLTensor* lv43, DLTensor* inputs_embeds, DLTensor* out0){

  
  
  using namespace fastertransformer;
  constexpr auto QuantOp = cutlass::WeightOnlyQuantOp::PER_COLUMN_SCALE_ONLY;

  int m = lv43->shape[0] * lv43->shape[1];
  int n = model_layers_0_self_attn_o_proj_weight_int8_01->shape[1] * 2;
  int k = model_layers_0_self_attn_o_proj_weight_int8_01->shape[0];

  auto func = tvm::runtime::Registry::Get("runtime.get_cuda_stream");
  ICHECK(func != nullptr);
  cudaStream_t stream = static_cast<cudaStream_t>((*func)().operator void*());
    
  gemm_fp16_int_bias_act<cutlass::uint4b_t, QuantOp>(static_cast<cutlass::half_t*>(lv43->data),
                static_cast<cutlass::uint4b_t*>(model_layers_0_self_attn_o_proj_weight_int8_01->data),
                static_cast<cutlass::half_t*>(model_layers_0_self_attn_o_proj_weight_float16_11->data),
                static_cast<cutlass::half_t*>(inputs_embeds->data),
                static_cast<cutlass::half_t*>(out0->data),
                "identity",
                m, n, k, k, 4096, nullptr, 0, stream);

}

int fused_decode7_relax_matmul_relax_add1_cutlass_wrapper_(DLTensor* arg0,
	DLTensor* arg1,
	DLTensor* arg2,
	DLTensor* arg3,
	DLTensor* out0) {
  fused_decode7_relax_matmul_relax_add1_cutlass_(arg0,
  arg1,
  arg2,
  arg3,
  out0);
  return 0;
}

#ifdef __cplusplus
extern "C" {
#endif
TVM_DLL int32_t fused_decode7_relax_matmul_relax_add1_cutlass(TVMValue* args, int* type_code, int num_args, TVMValue* out_value, int* out_type_code) {
  DLTensor* arg0 = (DLTensor*)(((TVMValue*)args)[0].v_handle);
  DLTensor* arg1 = (DLTensor*)(((TVMValue*)args)[1].v_handle);
  DLTensor* arg2 = (DLTensor*)(((TVMValue*)args)[2].v_handle);
  DLTensor* arg3 = (DLTensor*)(((TVMValue*)args)[3].v_handle);
  DLTensor* ret4 = (DLTensor*)(((TVMValue*)args)[4].v_handle);
  fused_decode7_relax_matmul_relax_add1_cutlass_wrapper_(arg0,arg1,arg2,arg3,ret4);
  return 0;
}
#ifdef __cplusplus
}
#endif

void fused_rms_norm1_cutlass_(DLTensor* lv1677, DLTensor* model_layers_0_input_layernorm_weight, DLTensor* out0){

  
    using data_type = cutlass::half_t;
    using namespace cutlass::layout;

    int M = 1;
    int N = 4096;
    cutlass::MatrixCoord size(M, N);
    auto layout_2D = RowMajor::packed(size);
    auto layout_channels = RowMajor::packed({1, N});

    cutlass::TensorRef<data_type, RowMajor> _input((data_type*)lv1677->data, layout_2D);
    cutlass::TensorRef<data_type, RowMajor> _weight((data_type*)model_layers_0_input_layernorm_weight->data, layout_channels);
    cutlass::TensorRef<data_type, RowMajor> _output((data_type*)out0->data, layout_2D);

    auto func = tvm::runtime::Registry::Get("runtime.get_cuda_stream");
    ICHECK(func != nullptr);
    cudaStream_t stream = static_cast<cudaStream_t>((*func)().operator void*());

    cutlass::rmsnorm(size, _output, _input, _weight, stream, 1e-05);
    
}

int fused_rms_norm1_cutlass_wrapper_(DLTensor* arg0,
	DLTensor* arg1,
	DLTensor* out0) {
  fused_rms_norm1_cutlass_(arg0,
  arg1,
  out0);
  return 0;
}

#ifdef __cplusplus
extern "C" {
#endif
TVM_DLL int32_t fused_rms_norm1_cutlass(TVMValue* args, int* type_code, int num_args, TVMValue* out_value, int* out_type_code) {
  DLTensor* arg0 = (DLTensor*)(((TVMValue*)args)[0].v_handle);
  DLTensor* arg1 = (DLTensor*)(((TVMValue*)args)[1].v_handle);
  DLTensor* ret2 = (DLTensor*)(((TVMValue*)args)[2].v_handle);
  fused_rms_norm1_cutlass_wrapper_(arg0,arg1,ret2);
  return 0;
}
#ifdef __cplusplus
}
#endif

void fused_decode9_relax_matmul_relax_add_cutlass_(DLTensor* model_layers_0_mlp_down_proj_weight_int8_0, DLTensor* model_layers_0_mlp_down_proj_weight_float16_1, DLTensor* lv1727, DLTensor* lv1719, DLTensor* out0){

  
  
  using namespace fastertransformer;
  constexpr auto QuantOp = cutlass::WeightOnlyQuantOp::PER_COLUMN_SCALE_ONLY;

  int m = 1;
  int n = model_layers_0_mlp_down_proj_weight_int8_0->shape[1] * 2;
  int k = model_layers_0_mlp_down_proj_weight_int8_0->shape[0];

  auto func = tvm::runtime::Registry::Get("runtime.get_cuda_stream");
  ICHECK(func != nullptr);
  cudaStream_t stream = static_cast<cudaStream_t>((*func)().operator void*());
    
  gemm_fp16_int_bias_act<cutlass::uint4b_t, QuantOp>(static_cast<cutlass::half_t*>(lv1727->data),
                static_cast<cutlass::uint4b_t*>(model_layers_0_mlp_down_proj_weight_int8_0->data),
                static_cast<cutlass::half_t*>(model_layers_0_mlp_down_proj_weight_float16_1->data),
                static_cast<cutlass::half_t*>(lv1719->data),
                static_cast<cutlass::half_t*>(out0->data),
                "identity",
                m, n, k, k, 0, nullptr, 0, stream);

}

int fused_decode9_relax_matmul_relax_add_cutlass_wrapper_(DLTensor* arg0,
	DLTensor* arg1,
	DLTensor* arg2,
	DLTensor* arg3,
	DLTensor* out0) {
  fused_decode9_relax_matmul_relax_add_cutlass_(arg0,
  arg1,
  arg2,
  arg3,
  out0);
  return 0;
}

#ifdef __cplusplus
extern "C" {
#endif
TVM_DLL int32_t fused_decode9_relax_matmul_relax_add_cutlass(TVMValue* args, int* type_code, int num_args, TVMValue* out_value, int* out_type_code) {
  DLTensor* arg0 = (DLTensor*)(((TVMValue*)args)[0].v_handle);
  DLTensor* arg1 = (DLTensor*)(((TVMValue*)args)[1].v_handle);
  DLTensor* arg2 = (DLTensor*)(((TVMValue*)args)[2].v_handle);
  DLTensor* arg3 = (DLTensor*)(((TVMValue*)args)[3].v_handle);
  DLTensor* ret4 = (DLTensor*)(((TVMValue*)args)[4].v_handle);
  fused_decode9_relax_matmul_relax_add_cutlass_wrapper_(arg0,arg1,arg2,arg3,ret4);
  return 0;
}
#ifdef __cplusplus
}
#endif
