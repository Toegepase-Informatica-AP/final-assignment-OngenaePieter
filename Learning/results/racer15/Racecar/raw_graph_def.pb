
C
global_step/initial_valueConst*
dtype0*
value	B : 
W
global_step
VariableV2*
	container *
dtype0*
shape: *
shared_name 
�
global_step/AssignAssignglobal_stepglobal_step/initial_value*
T0*
_class
loc:@global_step*
use_locking(*
validate_shape(
R
global_step/readIdentityglobal_step*
T0*
_class
loc:@global_step
;
steps_to_incrementPlaceholder*
dtype0*
shape: 
9
AddAddglobal_step/readsteps_to_increment*
T0
t
AssignAssignglobal_stepAdd*
T0*
_class
loc:@global_step*
use_locking(*
validate_shape(
L
vector_observationPlaceholder*
dtype0*
shape:���������
5

batch_sizePlaceholder*
dtype0*
shape:
:
sequence_lengthPlaceholder*
dtype0*
shape:
;
masksPlaceholder*
dtype0*
shape:���������
A
epsilonPlaceholder*
dtype0*
shape:���������
;
CastCastmasks*

DstT0*

SrcT0*
Truncate( 
M
#is_continuous_control/initial_valueConst*
dtype0*
value	B : 
a
is_continuous_control
VariableV2*
	container *
dtype0*
shape: *
shared_name 
�
is_continuous_control/AssignAssignis_continuous_control#is_continuous_control/initial_value*
T0*(
_class
loc:@is_continuous_control*
use_locking(*
validate_shape(
p
is_continuous_control/readIdentityis_continuous_control*
T0*(
_class
loc:@is_continuous_control
M
#trainer_major_version/initial_valueConst*
dtype0*
value	B : 
a
trainer_major_version
VariableV2*
	container *
dtype0*
shape: *
shared_name 
�
trainer_major_version/AssignAssigntrainer_major_version#trainer_major_version/initial_value*
T0*(
_class
loc:@trainer_major_version*
use_locking(*
validate_shape(
p
trainer_major_version/readIdentitytrainer_major_version*
T0*(
_class
loc:@trainer_major_version
M
#trainer_minor_version/initial_valueConst*
dtype0*
value	B :
a
trainer_minor_version
VariableV2*
	container *
dtype0*
shape: *
shared_name 
�
trainer_minor_version/AssignAssigntrainer_minor_version#trainer_minor_version/initial_value*
T0*(
_class
loc:@trainer_minor_version*
use_locking(*
validate_shape(
p
trainer_minor_version/readIdentitytrainer_minor_version*
T0*(
_class
loc:@trainer_minor_version
M
#trainer_patch_version/initial_valueConst*
dtype0*
value	B : 
a
trainer_patch_version
VariableV2*
	container *
dtype0*
shape: *
shared_name 
�
trainer_patch_version/AssignAssigntrainer_patch_version#trainer_patch_version/initial_value*
T0*(
_class
loc:@trainer_patch_version*
use_locking(*
validate_shape(
p
trainer_patch_version/readIdentitytrainer_patch_version*
T0*(
_class
loc:@trainer_patch_version
F
version_number/initial_valueConst*
dtype0*
value	B :
Z
version_number
VariableV2*
	container *
dtype0*
shape: *
shared_name 
�
version_number/AssignAssignversion_numberversion_number/initial_value*
T0*!
_class
loc:@version_number*
use_locking(*
validate_shape(
[
version_number/readIdentityversion_number*
T0*!
_class
loc:@version_number
C
memory_size/initial_valueConst*
dtype0*
value	B : 
W
memory_size
VariableV2*
	container *
dtype0*
shape: *
shared_name 
�
memory_size/AssignAssignmemory_sizememory_size/initial_value*
T0*
_class
loc:@memory_size*
use_locking(*
validate_shape(
R
memory_size/readIdentitymemory_size*
T0*
_class
loc:@memory_size
K
!action_output_shape/initial_valueConst*
dtype0*
value	B :
_
action_output_shape
VariableV2*
	container *
dtype0*
shape: *
shared_name 
�
action_output_shape/AssignAssignaction_output_shape!action_output_shape/initial_value*
T0*&
_class
loc:@action_output_shape*
use_locking(*
validate_shape(
j
action_output_shape/readIdentityaction_output_shape*
T0*&
_class
loc:@action_output_shape
�
Fpolicy/main_graph_0/hidden_0/kernel/Initializer/truncated_normal/shapeConst*6
_class,
*(loc:@policy/main_graph_0/hidden_0/kernel*
dtype0*
valueB"   �   
�
Epolicy/main_graph_0/hidden_0/kernel/Initializer/truncated_normal/meanConst*6
_class,
*(loc:@policy/main_graph_0/hidden_0/kernel*
dtype0*
valueB
 *    
�
Gpolicy/main_graph_0/hidden_0/kernel/Initializer/truncated_normal/stddevConst*6
_class,
*(loc:@policy/main_graph_0/hidden_0/kernel*
dtype0*
valueB
 *���=
�
Ppolicy/main_graph_0/hidden_0/kernel/Initializer/truncated_normal/TruncatedNormalTruncatedNormalFpolicy/main_graph_0/hidden_0/kernel/Initializer/truncated_normal/shape*
T0*6
_class,
*(loc:@policy/main_graph_0/hidden_0/kernel*
dtype0*
seed�0*
seed2 
�
Dpolicy/main_graph_0/hidden_0/kernel/Initializer/truncated_normal/mulMulPpolicy/main_graph_0/hidden_0/kernel/Initializer/truncated_normal/TruncatedNormalGpolicy/main_graph_0/hidden_0/kernel/Initializer/truncated_normal/stddev*
T0*6
_class,
*(loc:@policy/main_graph_0/hidden_0/kernel
�
@policy/main_graph_0/hidden_0/kernel/Initializer/truncated_normalAddDpolicy/main_graph_0/hidden_0/kernel/Initializer/truncated_normal/mulEpolicy/main_graph_0/hidden_0/kernel/Initializer/truncated_normal/mean*
T0*6
_class,
*(loc:@policy/main_graph_0/hidden_0/kernel
�
#policy/main_graph_0/hidden_0/kernel
VariableV2*6
_class,
*(loc:@policy/main_graph_0/hidden_0/kernel*
	container *
dtype0*
shape:	�*
shared_name 
�
*policy/main_graph_0/hidden_0/kernel/AssignAssign#policy/main_graph_0/hidden_0/kernel@policy/main_graph_0/hidden_0/kernel/Initializer/truncated_normal*
T0*6
_class,
*(loc:@policy/main_graph_0/hidden_0/kernel*
use_locking(*
validate_shape(
�
(policy/main_graph_0/hidden_0/kernel/readIdentity#policy/main_graph_0/hidden_0/kernel*
T0*6
_class,
*(loc:@policy/main_graph_0/hidden_0/kernel
�
3policy/main_graph_0/hidden_0/bias/Initializer/zerosConst*4
_class*
(&loc:@policy/main_graph_0/hidden_0/bias*
dtype0*
valueB�*    
�
!policy/main_graph_0/hidden_0/bias
VariableV2*4
_class*
(&loc:@policy/main_graph_0/hidden_0/bias*
	container *
dtype0*
shape:�*
shared_name 
�
(policy/main_graph_0/hidden_0/bias/AssignAssign!policy/main_graph_0/hidden_0/bias3policy/main_graph_0/hidden_0/bias/Initializer/zeros*
T0*4
_class*
(&loc:@policy/main_graph_0/hidden_0/bias*
use_locking(*
validate_shape(
�
&policy/main_graph_0/hidden_0/bias/readIdentity!policy/main_graph_0/hidden_0/bias*
T0*4
_class*
(&loc:@policy/main_graph_0/hidden_0/bias
�
#policy/main_graph_0/hidden_0/MatMulMatMulvector_observation(policy/main_graph_0/hidden_0/kernel/read*
T0*
transpose_a( *
transpose_b( 
�
$policy/main_graph_0/hidden_0/BiasAddBiasAdd#policy/main_graph_0/hidden_0/MatMul&policy/main_graph_0/hidden_0/bias/read*
T0*
data_formatNHWC
^
$policy/main_graph_0/hidden_0/SigmoidSigmoid$policy/main_graph_0/hidden_0/BiasAdd*
T0
|
 policy/main_graph_0/hidden_0/MulMul$policy/main_graph_0/hidden_0/BiasAdd$policy/main_graph_0/hidden_0/Sigmoid*
T0
�
Fpolicy/main_graph_0/hidden_1/kernel/Initializer/truncated_normal/shapeConst*6
_class,
*(loc:@policy/main_graph_0/hidden_1/kernel*
dtype0*
valueB"�   �   
�
Epolicy/main_graph_0/hidden_1/kernel/Initializer/truncated_normal/meanConst*6
_class,
*(loc:@policy/main_graph_0/hidden_1/kernel*
dtype0*
valueB
 *    
�
Gpolicy/main_graph_0/hidden_1/kernel/Initializer/truncated_normal/stddevConst*6
_class,
*(loc:@policy/main_graph_0/hidden_1/kernel*
dtype0*
valueB
 *���=
�
Ppolicy/main_graph_0/hidden_1/kernel/Initializer/truncated_normal/TruncatedNormalTruncatedNormalFpolicy/main_graph_0/hidden_1/kernel/Initializer/truncated_normal/shape*
T0*6
_class,
*(loc:@policy/main_graph_0/hidden_1/kernel*
dtype0*
seed�0*
seed2
�
Dpolicy/main_graph_0/hidden_1/kernel/Initializer/truncated_normal/mulMulPpolicy/main_graph_0/hidden_1/kernel/Initializer/truncated_normal/TruncatedNormalGpolicy/main_graph_0/hidden_1/kernel/Initializer/truncated_normal/stddev*
T0*6
_class,
*(loc:@policy/main_graph_0/hidden_1/kernel
�
@policy/main_graph_0/hidden_1/kernel/Initializer/truncated_normalAddDpolicy/main_graph_0/hidden_1/kernel/Initializer/truncated_normal/mulEpolicy/main_graph_0/hidden_1/kernel/Initializer/truncated_normal/mean*
T0*6
_class,
*(loc:@policy/main_graph_0/hidden_1/kernel
�
#policy/main_graph_0/hidden_1/kernel
VariableV2*6
_class,
*(loc:@policy/main_graph_0/hidden_1/kernel*
	container *
dtype0*
shape:
��*
shared_name 
�
*policy/main_graph_0/hidden_1/kernel/AssignAssign#policy/main_graph_0/hidden_1/kernel@policy/main_graph_0/hidden_1/kernel/Initializer/truncated_normal*
T0*6
_class,
*(loc:@policy/main_graph_0/hidden_1/kernel*
use_locking(*
validate_shape(
�
(policy/main_graph_0/hidden_1/kernel/readIdentity#policy/main_graph_0/hidden_1/kernel*
T0*6
_class,
*(loc:@policy/main_graph_0/hidden_1/kernel
�
3policy/main_graph_0/hidden_1/bias/Initializer/zerosConst*4
_class*
(&loc:@policy/main_graph_0/hidden_1/bias*
dtype0*
valueB�*    
�
!policy/main_graph_0/hidden_1/bias
VariableV2*4
_class*
(&loc:@policy/main_graph_0/hidden_1/bias*
	container *
dtype0*
shape:�*
shared_name 
�
(policy/main_graph_0/hidden_1/bias/AssignAssign!policy/main_graph_0/hidden_1/bias3policy/main_graph_0/hidden_1/bias/Initializer/zeros*
T0*4
_class*
(&loc:@policy/main_graph_0/hidden_1/bias*
use_locking(*
validate_shape(
�
&policy/main_graph_0/hidden_1/bias/readIdentity!policy/main_graph_0/hidden_1/bias*
T0*4
_class*
(&loc:@policy/main_graph_0/hidden_1/bias
�
#policy/main_graph_0/hidden_1/MatMulMatMul policy/main_graph_0/hidden_0/Mul(policy/main_graph_0/hidden_1/kernel/read*
T0*
transpose_a( *
transpose_b( 
�
$policy/main_graph_0/hidden_1/BiasAddBiasAdd#policy/main_graph_0/hidden_1/MatMul&policy/main_graph_0/hidden_1/bias/read*
T0*
data_formatNHWC
^
$policy/main_graph_0/hidden_1/SigmoidSigmoid$policy/main_graph_0/hidden_1/BiasAdd*
T0
|
 policy/main_graph_0/hidden_1/MulMul$policy/main_graph_0/hidden_1/BiasAdd$policy/main_graph_0/hidden_1/Sigmoid*
T0
F
action_masksPlaceholder*
dtype0*
shape:���������
�
6policy/dense/kernel/Initializer/truncated_normal/shapeConst*&
_class
loc:@policy/dense/kernel*
dtype0*
valueB"�      
�
5policy/dense/kernel/Initializer/truncated_normal/meanConst*&
_class
loc:@policy/dense/kernel*
dtype0*
valueB
 *    
�
7policy/dense/kernel/Initializer/truncated_normal/stddevConst*&
_class
loc:@policy/dense/kernel*
dtype0*
valueB
 *�$<
�
@policy/dense/kernel/Initializer/truncated_normal/TruncatedNormalTruncatedNormal6policy/dense/kernel/Initializer/truncated_normal/shape*
T0*&
_class
loc:@policy/dense/kernel*
dtype0*
seed�0*
seed2
�
4policy/dense/kernel/Initializer/truncated_normal/mulMul@policy/dense/kernel/Initializer/truncated_normal/TruncatedNormal7policy/dense/kernel/Initializer/truncated_normal/stddev*
T0*&
_class
loc:@policy/dense/kernel
�
0policy/dense/kernel/Initializer/truncated_normalAdd4policy/dense/kernel/Initializer/truncated_normal/mul5policy/dense/kernel/Initializer/truncated_normal/mean*
T0*&
_class
loc:@policy/dense/kernel
�
policy/dense/kernel
VariableV2*&
_class
loc:@policy/dense/kernel*
	container *
dtype0*
shape:	�*
shared_name 
�
policy/dense/kernel/AssignAssignpolicy/dense/kernel0policy/dense/kernel/Initializer/truncated_normal*
T0*&
_class
loc:@policy/dense/kernel*
use_locking(*
validate_shape(
j
policy/dense/kernel/readIdentitypolicy/dense/kernel*
T0*&
_class
loc:@policy/dense/kernel
�
policy_1/dense/MatMulMatMul policy/main_graph_0/hidden_1/Mulpolicy/dense/kernel/read*
T0*
transpose_a( *
transpose_b( 
�
8policy/dense_1/kernel/Initializer/truncated_normal/shapeConst*(
_class
loc:@policy/dense_1/kernel*
dtype0*
valueB"�      
�
7policy/dense_1/kernel/Initializer/truncated_normal/meanConst*(
_class
loc:@policy/dense_1/kernel*
dtype0*
valueB
 *    
�
9policy/dense_1/kernel/Initializer/truncated_normal/stddevConst*(
_class
loc:@policy/dense_1/kernel*
dtype0*
valueB
 *�$<
�
Bpolicy/dense_1/kernel/Initializer/truncated_normal/TruncatedNormalTruncatedNormal8policy/dense_1/kernel/Initializer/truncated_normal/shape*
T0*(
_class
loc:@policy/dense_1/kernel*
dtype0*
seed�0*
seed2
�
6policy/dense_1/kernel/Initializer/truncated_normal/mulMulBpolicy/dense_1/kernel/Initializer/truncated_normal/TruncatedNormal9policy/dense_1/kernel/Initializer/truncated_normal/stddev*
T0*(
_class
loc:@policy/dense_1/kernel
�
2policy/dense_1/kernel/Initializer/truncated_normalAdd6policy/dense_1/kernel/Initializer/truncated_normal/mul7policy/dense_1/kernel/Initializer/truncated_normal/mean*
T0*(
_class
loc:@policy/dense_1/kernel
�
policy/dense_1/kernel
VariableV2*(
_class
loc:@policy/dense_1/kernel*
	container *
dtype0*
shape:	�*
shared_name 
�
policy/dense_1/kernel/AssignAssignpolicy/dense_1/kernel2policy/dense_1/kernel/Initializer/truncated_normal*
T0*(
_class
loc:@policy/dense_1/kernel*
use_locking(*
validate_shape(
p
policy/dense_1/kernel/readIdentitypolicy/dense_1/kernel*
T0*(
_class
loc:@policy/dense_1/kernel
�
policy_1/dense_1/MatMulMatMul policy/main_graph_0/hidden_1/Mulpolicy/dense_1/kernel/read*
T0*
transpose_a( *
transpose_b( 
Q
policy_1/strided_slice/stackConst*
dtype0*
valueB"        
S
policy_1/strided_slice/stack_1Const*
dtype0*
valueB"       
S
policy_1/strided_slice/stack_2Const*
dtype0*
valueB"      
�
policy_1/strided_sliceStridedSliceaction_maskspolicy_1/strided_slice/stackpolicy_1/strided_slice/stack_1policy_1/strided_slice/stack_2*
Index0*
T0*

begin_mask*
ellipsis_mask *
end_mask*
new_axis_mask *
shrink_axis_mask 
S
policy_1/strided_slice_1/stackConst*
dtype0*
valueB"       
U
 policy_1/strided_slice_1/stack_1Const*
dtype0*
valueB"       
U
 policy_1/strided_slice_1/stack_2Const*
dtype0*
valueB"      
�
policy_1/strided_slice_1StridedSliceaction_maskspolicy_1/strided_slice_1/stack policy_1/strided_slice_1/stack_1 policy_1/strided_slice_1/stack_2*
Index0*
T0*

begin_mask*
ellipsis_mask *
end_mask*
new_axis_mask *
shrink_axis_mask 
;
policy_1/SoftmaxSoftmaxpolicy_1/dense/MatMul*
T0
;
policy_1/add/yConst*
dtype0*
valueB
 *���3
@
policy_1/addAddV2policy_1/Softmaxpolicy_1/add/y*
T0
B
policy_1/MulMulpolicy_1/addpolicy_1/strided_slice*
T0
?
policy_1/Softmax_1Softmaxpolicy_1/dense_1/MatMul*
T0
=
policy_1/add_1/yConst*
dtype0*
valueB
 *���3
F
policy_1/add_1AddV2policy_1/Softmax_1policy_1/add_1/y*
T0
H
policy_1/Mul_1Mulpolicy_1/add_1policy_1/strided_slice_1*
T0
H
policy_1/Sum/reduction_indicesConst*
dtype0*
value	B :
g
policy_1/SumSumpolicy_1/Mulpolicy_1/Sum/reduction_indices*
T0*

Tidx0*
	keep_dims(
@
policy_1/truedivRealDivpolicy_1/Mulpolicy_1/Sum*
T0
J
 policy_1/Sum_1/reduction_indicesConst*
dtype0*
value	B :
m
policy_1/Sum_1Sumpolicy_1/Mul_1 policy_1/Sum_1/reduction_indices*
T0*

Tidx0*
	keep_dims(
F
policy_1/truediv_1RealDivpolicy_1/Mul_1policy_1/Sum_1*
T0
=
policy_1/add_2/yConst*
dtype0*
valueB
 *���3
D
policy_1/add_2AddV2policy_1/truedivpolicy_1/add_2/y*
T0
,
policy_1/LogLogpolicy_1/add_2*
T0
V
,policy_1/multinomial/Multinomial/num_samplesConst*
dtype0*
value	B :
�
 policy_1/multinomial/MultinomialMultinomialpolicy_1/Log,policy_1/multinomial/Multinomial/num_samples*
T0*
output_dtype0	*
seed�0*
seed2
=
policy_1/add_3/yConst*
dtype0*
valueB
 *���3
F
policy_1/add_3AddV2policy_1/truediv_1policy_1/add_3/y*
T0
.
policy_1/Log_1Logpolicy_1/add_3*
T0
X
.policy_1/multinomial_1/Multinomial/num_samplesConst*
dtype0*
value	B :
�
"policy_1/multinomial_1/MultinomialMultinomialpolicy_1/Log_1.policy_1/multinomial_1/Multinomial/num_samples*
T0*
output_dtype0	*
seed�0*
seed2
>
policy_1/concat/axisConst*
dtype0*
value	B :
�
policy_1/concatConcatV2 policy_1/multinomial/Multinomial"policy_1/multinomial_1/Multinomialpolicy_1/concat/axis*
N*
T0	*

Tidx0
@
policy_1/concat_1/axisConst*
dtype0*
value	B :
y
policy_1/concat_1ConcatV2policy_1/truedivpolicy_1/truediv_1policy_1/concat_1/axis*
N*
T0*

Tidx0
=
policy_1/add_4/yConst*
dtype0*
valueB
 *���3
D
policy_1/add_4AddV2policy_1/truedivpolicy_1/add_4/y*
T0
.
policy_1/Log_2Logpolicy_1/add_4*
T0
=
policy_1/add_5/yConst*
dtype0*
valueB
 *���3
F
policy_1/add_5AddV2policy_1/truediv_1policy_1/add_5/y*
T0
.
policy_1/Log_3Logpolicy_1/add_5*
T0
@
policy_1/concat_2/axisConst*
dtype0*
value	B :
s
policy_1/concat_2ConcatV2policy_1/Log_2policy_1/Log_3policy_1/concat_2/axis*
N*
T0*

Tidx0
S
policy_1/strided_slice_2/stackConst*
dtype0*
valueB"        
U
 policy_1/strided_slice_2/stack_1Const*
dtype0*
valueB"       
U
 policy_1/strided_slice_2/stack_2Const*
dtype0*
valueB"      
�
policy_1/strided_slice_2StridedSlicepolicy_1/concatpolicy_1/strided_slice_2/stack policy_1/strided_slice_2/stack_1 policy_1/strided_slice_2/stack_2*
Index0*
T0	*

begin_mask*
ellipsis_mask *
end_mask*
new_axis_mask *
shrink_axis_mask
F
policy_1/one_hot/on_valueConst*
dtype0*
valueB
 *  �?
G
policy_1/one_hot/off_valueConst*
dtype0*
valueB
 *    
@
policy_1/one_hot/depthConst*
dtype0*
value	B :
�
policy_1/one_hotOneHotpolicy_1/strided_slice_2policy_1/one_hot/depthpolicy_1/one_hot/on_valuepolicy_1/one_hot/off_value*
T0*
TI0	*
axis���������
S
policy_1/strided_slice_3/stackConst*
dtype0*
valueB"       
U
 policy_1/strided_slice_3/stack_1Const*
dtype0*
valueB"       
U
 policy_1/strided_slice_3/stack_2Const*
dtype0*
valueB"      
�
policy_1/strided_slice_3StridedSlicepolicy_1/concatpolicy_1/strided_slice_3/stack policy_1/strided_slice_3/stack_1 policy_1/strided_slice_3/stack_2*
Index0*
T0	*

begin_mask*
ellipsis_mask *
end_mask*
new_axis_mask *
shrink_axis_mask
H
policy_1/one_hot_1/on_valueConst*
dtype0*
valueB
 *  �?
I
policy_1/one_hot_1/off_valueConst*
dtype0*
valueB
 *    
B
policy_1/one_hot_1/depthConst*
dtype0*
value	B :
�
policy_1/one_hot_1OneHotpolicy_1/strided_slice_3policy_1/one_hot_1/depthpolicy_1/one_hot_1/on_valuepolicy_1/one_hot_1/off_value*
T0*
TI0	*
axis���������
@
policy_1/concat_3/axisConst*
dtype0*
value	B :
y
policy_1/concat_3ConcatV2policy_1/one_hotpolicy_1/one_hot_1policy_1/concat_3/axis*
N*
T0*

Tidx0
S
policy_1/strided_slice_4/stackConst*
dtype0*
valueB"        
U
 policy_1/strided_slice_4/stack_1Const*
dtype0*
valueB"       
U
 policy_1/strided_slice_4/stack_2Const*
dtype0*
valueB"      
�
policy_1/strided_slice_4StridedSlicepolicy_1/concat_2policy_1/strided_slice_4/stack policy_1/strided_slice_4/stack_1 policy_1/strided_slice_4/stack_2*
Index0*
T0*

begin_mask*
ellipsis_mask *
end_mask*
new_axis_mask *
shrink_axis_mask 
@
policy_1/Softmax_2Softmaxpolicy_1/strided_slice_4*
T0
S
policy_1/strided_slice_5/stackConst*
dtype0*
valueB"        
U
 policy_1/strided_slice_5/stack_1Const*
dtype0*
valueB"       
U
 policy_1/strided_slice_5/stack_2Const*
dtype0*
valueB"      
�
policy_1/strided_slice_5StridedSlicepolicy_1/concat_2policy_1/strided_slice_5/stack policy_1/strided_slice_5/stack_1 policy_1/strided_slice_5/stack_2*
Index0*
T0*

begin_mask*
ellipsis_mask *
end_mask*
new_axis_mask *
shrink_axis_mask 
Y
/policy_1/softmax_cross_entropy_with_logits/RankConst*
dtype0*
value	B :
l
0policy_1/softmax_cross_entropy_with_logits/ShapeShapepolicy_1/strided_slice_5*
T0*
out_type0
[
1policy_1/softmax_cross_entropy_with_logits/Rank_1Const*
dtype0*
value	B :
n
2policy_1/softmax_cross_entropy_with_logits/Shape_1Shapepolicy_1/strided_slice_5*
T0*
out_type0
Z
0policy_1/softmax_cross_entropy_with_logits/Sub/yConst*
dtype0*
value	B :
�
.policy_1/softmax_cross_entropy_with_logits/SubSub1policy_1/softmax_cross_entropy_with_logits/Rank_10policy_1/softmax_cross_entropy_with_logits/Sub/y*
T0
�
6policy_1/softmax_cross_entropy_with_logits/Slice/beginPack.policy_1/softmax_cross_entropy_with_logits/Sub*
N*
T0*

axis 
c
5policy_1/softmax_cross_entropy_with_logits/Slice/sizeConst*
dtype0*
valueB:
�
0policy_1/softmax_cross_entropy_with_logits/SliceSlice2policy_1/softmax_cross_entropy_with_logits/Shape_16policy_1/softmax_cross_entropy_with_logits/Slice/begin5policy_1/softmax_cross_entropy_with_logits/Slice/size*
Index0*
T0
q
:policy_1/softmax_cross_entropy_with_logits/concat/values_0Const*
dtype0*
valueB:
���������
`
6policy_1/softmax_cross_entropy_with_logits/concat/axisConst*
dtype0*
value	B : 
�
1policy_1/softmax_cross_entropy_with_logits/concatConcatV2:policy_1/softmax_cross_entropy_with_logits/concat/values_00policy_1/softmax_cross_entropy_with_logits/Slice6policy_1/softmax_cross_entropy_with_logits/concat/axis*
N*
T0*

Tidx0
�
2policy_1/softmax_cross_entropy_with_logits/ReshapeReshapepolicy_1/strided_slice_51policy_1/softmax_cross_entropy_with_logits/concat*
T0*
Tshape0
[
1policy_1/softmax_cross_entropy_with_logits/Rank_2Const*
dtype0*
value	B :
h
2policy_1/softmax_cross_entropy_with_logits/Shape_2Shapepolicy_1/Softmax_2*
T0*
out_type0
\
2policy_1/softmax_cross_entropy_with_logits/Sub_1/yConst*
dtype0*
value	B :
�
0policy_1/softmax_cross_entropy_with_logits/Sub_1Sub1policy_1/softmax_cross_entropy_with_logits/Rank_22policy_1/softmax_cross_entropy_with_logits/Sub_1/y*
T0
�
8policy_1/softmax_cross_entropy_with_logits/Slice_1/beginPack0policy_1/softmax_cross_entropy_with_logits/Sub_1*
N*
T0*

axis 
e
7policy_1/softmax_cross_entropy_with_logits/Slice_1/sizeConst*
dtype0*
valueB:
�
2policy_1/softmax_cross_entropy_with_logits/Slice_1Slice2policy_1/softmax_cross_entropy_with_logits/Shape_28policy_1/softmax_cross_entropy_with_logits/Slice_1/begin7policy_1/softmax_cross_entropy_with_logits/Slice_1/size*
Index0*
T0
s
<policy_1/softmax_cross_entropy_with_logits/concat_1/values_0Const*
dtype0*
valueB:
���������
b
8policy_1/softmax_cross_entropy_with_logits/concat_1/axisConst*
dtype0*
value	B : 
�
3policy_1/softmax_cross_entropy_with_logits/concat_1ConcatV2<policy_1/softmax_cross_entropy_with_logits/concat_1/values_02policy_1/softmax_cross_entropy_with_logits/Slice_18policy_1/softmax_cross_entropy_with_logits/concat_1/axis*
N*
T0*

Tidx0
�
4policy_1/softmax_cross_entropy_with_logits/Reshape_1Reshapepolicy_1/Softmax_23policy_1/softmax_cross_entropy_with_logits/concat_1*
T0*
Tshape0
�
*policy_1/softmax_cross_entropy_with_logitsSoftmaxCrossEntropyWithLogits2policy_1/softmax_cross_entropy_with_logits/Reshape4policy_1/softmax_cross_entropy_with_logits/Reshape_1*
T0
\
2policy_1/softmax_cross_entropy_with_logits/Sub_2/yConst*
dtype0*
value	B :
�
0policy_1/softmax_cross_entropy_with_logits/Sub_2Sub/policy_1/softmax_cross_entropy_with_logits/Rank2policy_1/softmax_cross_entropy_with_logits/Sub_2/y*
T0
f
8policy_1/softmax_cross_entropy_with_logits/Slice_2/beginConst*
dtype0*
valueB: 
�
7policy_1/softmax_cross_entropy_with_logits/Slice_2/sizePack0policy_1/softmax_cross_entropy_with_logits/Sub_2*
N*
T0*

axis 
�
2policy_1/softmax_cross_entropy_with_logits/Slice_2Slice0policy_1/softmax_cross_entropy_with_logits/Shape8policy_1/softmax_cross_entropy_with_logits/Slice_2/begin7policy_1/softmax_cross_entropy_with_logits/Slice_2/size*
Index0*
T0
�
4policy_1/softmax_cross_entropy_with_logits/Reshape_2Reshape*policy_1/softmax_cross_entropy_with_logits2policy_1/softmax_cross_entropy_with_logits/Slice_2*
T0*
Tshape0
S
policy_1/strided_slice_6/stackConst*
dtype0*
valueB"       
U
 policy_1/strided_slice_6/stack_1Const*
dtype0*
valueB"       
U
 policy_1/strided_slice_6/stack_2Const*
dtype0*
valueB"      
�
policy_1/strided_slice_6StridedSlicepolicy_1/concat_2policy_1/strided_slice_6/stack policy_1/strided_slice_6/stack_1 policy_1/strided_slice_6/stack_2*
Index0*
T0*

begin_mask*
ellipsis_mask *
end_mask*
new_axis_mask *
shrink_axis_mask 
@
policy_1/Softmax_3Softmaxpolicy_1/strided_slice_6*
T0
S
policy_1/strided_slice_7/stackConst*
dtype0*
valueB"       
U
 policy_1/strided_slice_7/stack_1Const*
dtype0*
valueB"       
U
 policy_1/strided_slice_7/stack_2Const*
dtype0*
valueB"      
�
policy_1/strided_slice_7StridedSlicepolicy_1/concat_2policy_1/strided_slice_7/stack policy_1/strided_slice_7/stack_1 policy_1/strided_slice_7/stack_2*
Index0*
T0*

begin_mask*
ellipsis_mask *
end_mask*
new_axis_mask *
shrink_axis_mask 
[
1policy_1/softmax_cross_entropy_with_logits_1/RankConst*
dtype0*
value	B :
n
2policy_1/softmax_cross_entropy_with_logits_1/ShapeShapepolicy_1/strided_slice_7*
T0*
out_type0
]
3policy_1/softmax_cross_entropy_with_logits_1/Rank_1Const*
dtype0*
value	B :
p
4policy_1/softmax_cross_entropy_with_logits_1/Shape_1Shapepolicy_1/strided_slice_7*
T0*
out_type0
\
2policy_1/softmax_cross_entropy_with_logits_1/Sub/yConst*
dtype0*
value	B :
�
0policy_1/softmax_cross_entropy_with_logits_1/SubSub3policy_1/softmax_cross_entropy_with_logits_1/Rank_12policy_1/softmax_cross_entropy_with_logits_1/Sub/y*
T0
�
8policy_1/softmax_cross_entropy_with_logits_1/Slice/beginPack0policy_1/softmax_cross_entropy_with_logits_1/Sub*
N*
T0*

axis 
e
7policy_1/softmax_cross_entropy_with_logits_1/Slice/sizeConst*
dtype0*
valueB:
�
2policy_1/softmax_cross_entropy_with_logits_1/SliceSlice4policy_1/softmax_cross_entropy_with_logits_1/Shape_18policy_1/softmax_cross_entropy_with_logits_1/Slice/begin7policy_1/softmax_cross_entropy_with_logits_1/Slice/size*
Index0*
T0
s
<policy_1/softmax_cross_entropy_with_logits_1/concat/values_0Const*
dtype0*
valueB:
���������
b
8policy_1/softmax_cross_entropy_with_logits_1/concat/axisConst*
dtype0*
value	B : 
�
3policy_1/softmax_cross_entropy_with_logits_1/concatConcatV2<policy_1/softmax_cross_entropy_with_logits_1/concat/values_02policy_1/softmax_cross_entropy_with_logits_1/Slice8policy_1/softmax_cross_entropy_with_logits_1/concat/axis*
N*
T0*

Tidx0
�
4policy_1/softmax_cross_entropy_with_logits_1/ReshapeReshapepolicy_1/strided_slice_73policy_1/softmax_cross_entropy_with_logits_1/concat*
T0*
Tshape0
]
3policy_1/softmax_cross_entropy_with_logits_1/Rank_2Const*
dtype0*
value	B :
j
4policy_1/softmax_cross_entropy_with_logits_1/Shape_2Shapepolicy_1/Softmax_3*
T0*
out_type0
^
4policy_1/softmax_cross_entropy_with_logits_1/Sub_1/yConst*
dtype0*
value	B :
�
2policy_1/softmax_cross_entropy_with_logits_1/Sub_1Sub3policy_1/softmax_cross_entropy_with_logits_1/Rank_24policy_1/softmax_cross_entropy_with_logits_1/Sub_1/y*
T0
�
:policy_1/softmax_cross_entropy_with_logits_1/Slice_1/beginPack2policy_1/softmax_cross_entropy_with_logits_1/Sub_1*
N*
T0*

axis 
g
9policy_1/softmax_cross_entropy_with_logits_1/Slice_1/sizeConst*
dtype0*
valueB:
�
4policy_1/softmax_cross_entropy_with_logits_1/Slice_1Slice4policy_1/softmax_cross_entropy_with_logits_1/Shape_2:policy_1/softmax_cross_entropy_with_logits_1/Slice_1/begin9policy_1/softmax_cross_entropy_with_logits_1/Slice_1/size*
Index0*
T0
u
>policy_1/softmax_cross_entropy_with_logits_1/concat_1/values_0Const*
dtype0*
valueB:
���������
d
:policy_1/softmax_cross_entropy_with_logits_1/concat_1/axisConst*
dtype0*
value	B : 
�
5policy_1/softmax_cross_entropy_with_logits_1/concat_1ConcatV2>policy_1/softmax_cross_entropy_with_logits_1/concat_1/values_04policy_1/softmax_cross_entropy_with_logits_1/Slice_1:policy_1/softmax_cross_entropy_with_logits_1/concat_1/axis*
N*
T0*

Tidx0
�
6policy_1/softmax_cross_entropy_with_logits_1/Reshape_1Reshapepolicy_1/Softmax_35policy_1/softmax_cross_entropy_with_logits_1/concat_1*
T0*
Tshape0
�
,policy_1/softmax_cross_entropy_with_logits_1SoftmaxCrossEntropyWithLogits4policy_1/softmax_cross_entropy_with_logits_1/Reshape6policy_1/softmax_cross_entropy_with_logits_1/Reshape_1*
T0
^
4policy_1/softmax_cross_entropy_with_logits_1/Sub_2/yConst*
dtype0*
value	B :
�
2policy_1/softmax_cross_entropy_with_logits_1/Sub_2Sub1policy_1/softmax_cross_entropy_with_logits_1/Rank4policy_1/softmax_cross_entropy_with_logits_1/Sub_2/y*
T0
h
:policy_1/softmax_cross_entropy_with_logits_1/Slice_2/beginConst*
dtype0*
valueB: 
�
9policy_1/softmax_cross_entropy_with_logits_1/Slice_2/sizePack2policy_1/softmax_cross_entropy_with_logits_1/Sub_2*
N*
T0*

axis 
�
4policy_1/softmax_cross_entropy_with_logits_1/Slice_2Slice2policy_1/softmax_cross_entropy_with_logits_1/Shape:policy_1/softmax_cross_entropy_with_logits_1/Slice_2/begin9policy_1/softmax_cross_entropy_with_logits_1/Slice_2/size*
Index0*
T0
�
6policy_1/softmax_cross_entropy_with_logits_1/Reshape_2Reshape,policy_1/softmax_cross_entropy_with_logits_14policy_1/softmax_cross_entropy_with_logits_1/Slice_2*
T0*
Tshape0
�
policy_1/stackPack4policy_1/softmax_cross_entropy_with_logits/Reshape_26policy_1/softmax_cross_entropy_with_logits_1/Reshape_2*
N*
T0*

axis
J
 policy_1/Sum_2/reduction_indicesConst*
dtype0*
value	B :
m
policy_1/Sum_2Sumpolicy_1/stack policy_1/Sum_2/reduction_indices*
T0*

Tidx0*
	keep_dims( 
S
policy_1/strided_slice_8/stackConst*
dtype0*
valueB"        
U
 policy_1/strided_slice_8/stack_1Const*
dtype0*
valueB"       
U
 policy_1/strided_slice_8/stack_2Const*
dtype0*
valueB"      
�
policy_1/strided_slice_8StridedSlicepolicy_1/concat_3policy_1/strided_slice_8/stack policy_1/strided_slice_8/stack_1 policy_1/strided_slice_8/stack_2*
Index0*
T0*

begin_mask*
ellipsis_mask *
end_mask*
new_axis_mask *
shrink_axis_mask 
S
policy_1/strided_slice_9/stackConst*
dtype0*
valueB"        
U
 policy_1/strided_slice_9/stack_1Const*
dtype0*
valueB"       
U
 policy_1/strided_slice_9/stack_2Const*
dtype0*
valueB"      
�
policy_1/strided_slice_9StridedSlicepolicy_1/concat_2policy_1/strided_slice_9/stack policy_1/strided_slice_9/stack_1 policy_1/strided_slice_9/stack_2*
Index0*
T0*

begin_mask*
ellipsis_mask *
end_mask*
new_axis_mask *
shrink_axis_mask 
[
1policy_1/softmax_cross_entropy_with_logits_2/RankConst*
dtype0*
value	B :
n
2policy_1/softmax_cross_entropy_with_logits_2/ShapeShapepolicy_1/strided_slice_9*
T0*
out_type0
]
3policy_1/softmax_cross_entropy_with_logits_2/Rank_1Const*
dtype0*
value	B :
p
4policy_1/softmax_cross_entropy_with_logits_2/Shape_1Shapepolicy_1/strided_slice_9*
T0*
out_type0
\
2policy_1/softmax_cross_entropy_with_logits_2/Sub/yConst*
dtype0*
value	B :
�
0policy_1/softmax_cross_entropy_with_logits_2/SubSub3policy_1/softmax_cross_entropy_with_logits_2/Rank_12policy_1/softmax_cross_entropy_with_logits_2/Sub/y*
T0
�
8policy_1/softmax_cross_entropy_with_logits_2/Slice/beginPack0policy_1/softmax_cross_entropy_with_logits_2/Sub*
N*
T0*

axis 
e
7policy_1/softmax_cross_entropy_with_logits_2/Slice/sizeConst*
dtype0*
valueB:
�
2policy_1/softmax_cross_entropy_with_logits_2/SliceSlice4policy_1/softmax_cross_entropy_with_logits_2/Shape_18policy_1/softmax_cross_entropy_with_logits_2/Slice/begin7policy_1/softmax_cross_entropy_with_logits_2/Slice/size*
Index0*
T0
s
<policy_1/softmax_cross_entropy_with_logits_2/concat/values_0Const*
dtype0*
valueB:
���������
b
8policy_1/softmax_cross_entropy_with_logits_2/concat/axisConst*
dtype0*
value	B : 
�
3policy_1/softmax_cross_entropy_with_logits_2/concatConcatV2<policy_1/softmax_cross_entropy_with_logits_2/concat/values_02policy_1/softmax_cross_entropy_with_logits_2/Slice8policy_1/softmax_cross_entropy_with_logits_2/concat/axis*
N*
T0*

Tidx0
�
4policy_1/softmax_cross_entropy_with_logits_2/ReshapeReshapepolicy_1/strided_slice_93policy_1/softmax_cross_entropy_with_logits_2/concat*
T0*
Tshape0
]
3policy_1/softmax_cross_entropy_with_logits_2/Rank_2Const*
dtype0*
value	B :
p
4policy_1/softmax_cross_entropy_with_logits_2/Shape_2Shapepolicy_1/strided_slice_8*
T0*
out_type0
^
4policy_1/softmax_cross_entropy_with_logits_2/Sub_1/yConst*
dtype0*
value	B :
�
2policy_1/softmax_cross_entropy_with_logits_2/Sub_1Sub3policy_1/softmax_cross_entropy_with_logits_2/Rank_24policy_1/softmax_cross_entropy_with_logits_2/Sub_1/y*
T0
�
:policy_1/softmax_cross_entropy_with_logits_2/Slice_1/beginPack2policy_1/softmax_cross_entropy_with_logits_2/Sub_1*
N*
T0*

axis 
g
9policy_1/softmax_cross_entropy_with_logits_2/Slice_1/sizeConst*
dtype0*
valueB:
�
4policy_1/softmax_cross_entropy_with_logits_2/Slice_1Slice4policy_1/softmax_cross_entropy_with_logits_2/Shape_2:policy_1/softmax_cross_entropy_with_logits_2/Slice_1/begin9policy_1/softmax_cross_entropy_with_logits_2/Slice_1/size*
Index0*
T0
u
>policy_1/softmax_cross_entropy_with_logits_2/concat_1/values_0Const*
dtype0*
valueB:
���������
d
:policy_1/softmax_cross_entropy_with_logits_2/concat_1/axisConst*
dtype0*
value	B : 
�
5policy_1/softmax_cross_entropy_with_logits_2/concat_1ConcatV2>policy_1/softmax_cross_entropy_with_logits_2/concat_1/values_04policy_1/softmax_cross_entropy_with_logits_2/Slice_1:policy_1/softmax_cross_entropy_with_logits_2/concat_1/axis*
N*
T0*

Tidx0
�
6policy_1/softmax_cross_entropy_with_logits_2/Reshape_1Reshapepolicy_1/strided_slice_85policy_1/softmax_cross_entropy_with_logits_2/concat_1*
T0*
Tshape0
�
,policy_1/softmax_cross_entropy_with_logits_2SoftmaxCrossEntropyWithLogits4policy_1/softmax_cross_entropy_with_logits_2/Reshape6policy_1/softmax_cross_entropy_with_logits_2/Reshape_1*
T0
^
4policy_1/softmax_cross_entropy_with_logits_2/Sub_2/yConst*
dtype0*
value	B :
�
2policy_1/softmax_cross_entropy_with_logits_2/Sub_2Sub1policy_1/softmax_cross_entropy_with_logits_2/Rank4policy_1/softmax_cross_entropy_with_logits_2/Sub_2/y*
T0
h
:policy_1/softmax_cross_entropy_with_logits_2/Slice_2/beginConst*
dtype0*
valueB: 
�
9policy_1/softmax_cross_entropy_with_logits_2/Slice_2/sizePack2policy_1/softmax_cross_entropy_with_logits_2/Sub_2*
N*
T0*

axis 
�
4policy_1/softmax_cross_entropy_with_logits_2/Slice_2Slice2policy_1/softmax_cross_entropy_with_logits_2/Shape:policy_1/softmax_cross_entropy_with_logits_2/Slice_2/begin9policy_1/softmax_cross_entropy_with_logits_2/Slice_2/size*
Index0*
T0
�
6policy_1/softmax_cross_entropy_with_logits_2/Reshape_2Reshape,policy_1/softmax_cross_entropy_with_logits_24policy_1/softmax_cross_entropy_with_logits_2/Slice_2*
T0*
Tshape0
T
policy_1/NegNeg6policy_1/softmax_cross_entropy_with_logits_2/Reshape_2*
T0
T
policy_1/strided_slice_10/stackConst*
dtype0*
valueB"       
V
!policy_1/strided_slice_10/stack_1Const*
dtype0*
valueB"       
V
!policy_1/strided_slice_10/stack_2Const*
dtype0*
valueB"      
�
policy_1/strided_slice_10StridedSlicepolicy_1/concat_3policy_1/strided_slice_10/stack!policy_1/strided_slice_10/stack_1!policy_1/strided_slice_10/stack_2*
Index0*
T0*

begin_mask*
ellipsis_mask *
end_mask*
new_axis_mask *
shrink_axis_mask 
T
policy_1/strided_slice_11/stackConst*
dtype0*
valueB"       
V
!policy_1/strided_slice_11/stack_1Const*
dtype0*
valueB"       
V
!policy_1/strided_slice_11/stack_2Const*
dtype0*
valueB"      
�
policy_1/strided_slice_11StridedSlicepolicy_1/concat_2policy_1/strided_slice_11/stack!policy_1/strided_slice_11/stack_1!policy_1/strided_slice_11/stack_2*
Index0*
T0*

begin_mask*
ellipsis_mask *
end_mask*
new_axis_mask *
shrink_axis_mask 
[
1policy_1/softmax_cross_entropy_with_logits_3/RankConst*
dtype0*
value	B :
o
2policy_1/softmax_cross_entropy_with_logits_3/ShapeShapepolicy_1/strided_slice_11*
T0*
out_type0
]
3policy_1/softmax_cross_entropy_with_logits_3/Rank_1Const*
dtype0*
value	B :
q
4policy_1/softmax_cross_entropy_with_logits_3/Shape_1Shapepolicy_1/strided_slice_11*
T0*
out_type0
\
2policy_1/softmax_cross_entropy_with_logits_3/Sub/yConst*
dtype0*
value	B :
�
0policy_1/softmax_cross_entropy_with_logits_3/SubSub3policy_1/softmax_cross_entropy_with_logits_3/Rank_12policy_1/softmax_cross_entropy_with_logits_3/Sub/y*
T0
�
8policy_1/softmax_cross_entropy_with_logits_3/Slice/beginPack0policy_1/softmax_cross_entropy_with_logits_3/Sub*
N*
T0*

axis 
e
7policy_1/softmax_cross_entropy_with_logits_3/Slice/sizeConst*
dtype0*
valueB:
�
2policy_1/softmax_cross_entropy_with_logits_3/SliceSlice4policy_1/softmax_cross_entropy_with_logits_3/Shape_18policy_1/softmax_cross_entropy_with_logits_3/Slice/begin7policy_1/softmax_cross_entropy_with_logits_3/Slice/size*
Index0*
T0
s
<policy_1/softmax_cross_entropy_with_logits_3/concat/values_0Const*
dtype0*
valueB:
���������
b
8policy_1/softmax_cross_entropy_with_logits_3/concat/axisConst*
dtype0*
value	B : 
�
3policy_1/softmax_cross_entropy_with_logits_3/concatConcatV2<policy_1/softmax_cross_entropy_with_logits_3/concat/values_02policy_1/softmax_cross_entropy_with_logits_3/Slice8policy_1/softmax_cross_entropy_with_logits_3/concat/axis*
N*
T0*

Tidx0
�
4policy_1/softmax_cross_entropy_with_logits_3/ReshapeReshapepolicy_1/strided_slice_113policy_1/softmax_cross_entropy_with_logits_3/concat*
T0*
Tshape0
]
3policy_1/softmax_cross_entropy_with_logits_3/Rank_2Const*
dtype0*
value	B :
q
4policy_1/softmax_cross_entropy_with_logits_3/Shape_2Shapepolicy_1/strided_slice_10*
T0*
out_type0
^
4policy_1/softmax_cross_entropy_with_logits_3/Sub_1/yConst*
dtype0*
value	B :
�
2policy_1/softmax_cross_entropy_with_logits_3/Sub_1Sub3policy_1/softmax_cross_entropy_with_logits_3/Rank_24policy_1/softmax_cross_entropy_with_logits_3/Sub_1/y*
T0
�
:policy_1/softmax_cross_entropy_with_logits_3/Slice_1/beginPack2policy_1/softmax_cross_entropy_with_logits_3/Sub_1*
N*
T0*

axis 
g
9policy_1/softmax_cross_entropy_with_logits_3/Slice_1/sizeConst*
dtype0*
valueB:
�
4policy_1/softmax_cross_entropy_with_logits_3/Slice_1Slice4policy_1/softmax_cross_entropy_with_logits_3/Shape_2:policy_1/softmax_cross_entropy_with_logits_3/Slice_1/begin9policy_1/softmax_cross_entropy_with_logits_3/Slice_1/size*
Index0*
T0
u
>policy_1/softmax_cross_entropy_with_logits_3/concat_1/values_0Const*
dtype0*
valueB:
���������
d
:policy_1/softmax_cross_entropy_with_logits_3/concat_1/axisConst*
dtype0*
value	B : 
�
5policy_1/softmax_cross_entropy_with_logits_3/concat_1ConcatV2>policy_1/softmax_cross_entropy_with_logits_3/concat_1/values_04policy_1/softmax_cross_entropy_with_logits_3/Slice_1:policy_1/softmax_cross_entropy_with_logits_3/concat_1/axis*
N*
T0*

Tidx0
�
6policy_1/softmax_cross_entropy_with_logits_3/Reshape_1Reshapepolicy_1/strided_slice_105policy_1/softmax_cross_entropy_with_logits_3/concat_1*
T0*
Tshape0
�
,policy_1/softmax_cross_entropy_with_logits_3SoftmaxCrossEntropyWithLogits4policy_1/softmax_cross_entropy_with_logits_3/Reshape6policy_1/softmax_cross_entropy_with_logits_3/Reshape_1*
T0
^
4policy_1/softmax_cross_entropy_with_logits_3/Sub_2/yConst*
dtype0*
value	B :
�
2policy_1/softmax_cross_entropy_with_logits_3/Sub_2Sub1policy_1/softmax_cross_entropy_with_logits_3/Rank4policy_1/softmax_cross_entropy_with_logits_3/Sub_2/y*
T0
h
:policy_1/softmax_cross_entropy_with_logits_3/Slice_2/beginConst*
dtype0*
valueB: 
�
9policy_1/softmax_cross_entropy_with_logits_3/Slice_2/sizePack2policy_1/softmax_cross_entropy_with_logits_3/Sub_2*
N*
T0*

axis 
�
4policy_1/softmax_cross_entropy_with_logits_3/Slice_2Slice2policy_1/softmax_cross_entropy_with_logits_3/Shape:policy_1/softmax_cross_entropy_with_logits_3/Slice_2/begin9policy_1/softmax_cross_entropy_with_logits_3/Slice_2/size*
Index0*
T0
�
6policy_1/softmax_cross_entropy_with_logits_3/Reshape_2Reshape,policy_1/softmax_cross_entropy_with_logits_34policy_1/softmax_cross_entropy_with_logits_3/Slice_2*
T0*
Tshape0
V
policy_1/Neg_1Neg6policy_1/softmax_cross_entropy_with_logits_3/Reshape_2*
T0
T
policy_1/stack_1Packpolicy_1/Negpolicy_1/Neg_1*
N*
T0*

axis
J
 policy_1/Sum_3/reduction_indicesConst*
dtype0*
value	B :
o
policy_1/Sum_3Sumpolicy_1/stack_1 policy_1/Sum_3/reduction_indices*
T0*

Tidx0*
	keep_dims(
.
actionIdentitypolicy_1/concat_2*
T0
8
StopGradientStopGradientpolicy_1/concat_3*
T0
A
save/filename/inputConst*
dtype0*
valueB Bmodel
V
save/filenamePlaceholderWithDefaultsave/filename/input*
dtype0*
shape: 
M

save/ConstPlaceholderWithDefaultsave/filename*
dtype0*
shape: 
�
save/SaveV2/tensor_namesConst*
dtype0*�
value�B�Baction_output_shapeBglobal_stepBis_continuous_controlBmemory_sizeBpolicy/dense/kernelBpolicy/dense_1/kernelB!policy/main_graph_0/hidden_0/biasB#policy/main_graph_0/hidden_0/kernelB!policy/main_graph_0/hidden_1/biasB#policy/main_graph_0/hidden_1/kernelBtrainer_major_versionBtrainer_minor_versionBtrainer_patch_versionBversion_number
c
save/SaveV2/shape_and_slicesConst*
dtype0*/
value&B$B B B B B B B B B B B B B B 
�
save/SaveV2SaveV2
save/Constsave/SaveV2/tensor_namessave/SaveV2/shape_and_slicesaction_output_shapeglobal_stepis_continuous_controlmemory_sizepolicy/dense/kernelpolicy/dense_1/kernel!policy/main_graph_0/hidden_0/bias#policy/main_graph_0/hidden_0/kernel!policy/main_graph_0/hidden_1/bias#policy/main_graph_0/hidden_1/kerneltrainer_major_versiontrainer_minor_versiontrainer_patch_versionversion_number*
dtypes
2
e
save/control_dependencyIdentity
save/Const^save/SaveV2*
T0*
_class
loc:@save/Const
�
save/RestoreV2/tensor_namesConst"/device:CPU:0*
dtype0*�
value�B�Baction_output_shapeBglobal_stepBis_continuous_controlBmemory_sizeBpolicy/dense/kernelBpolicy/dense_1/kernelB!policy/main_graph_0/hidden_0/biasB#policy/main_graph_0/hidden_0/kernelB!policy/main_graph_0/hidden_1/biasB#policy/main_graph_0/hidden_1/kernelBtrainer_major_versionBtrainer_minor_versionBtrainer_patch_versionBversion_number
u
save/RestoreV2/shape_and_slicesConst"/device:CPU:0*
dtype0*/
value&B$B B B B B B B B B B B B B B 
�
save/RestoreV2	RestoreV2
save/Constsave/RestoreV2/tensor_namessave/RestoreV2/shape_and_slices"/device:CPU:0*
dtypes
2
�
save/AssignAssignaction_output_shapesave/RestoreV2*
T0*&
_class
loc:@action_output_shape*
use_locking(*
validate_shape(
�
save/Assign_1Assignglobal_stepsave/RestoreV2:1*
T0*
_class
loc:@global_step*
use_locking(*
validate_shape(
�
save/Assign_2Assignis_continuous_controlsave/RestoreV2:2*
T0*(
_class
loc:@is_continuous_control*
use_locking(*
validate_shape(
�
save/Assign_3Assignmemory_sizesave/RestoreV2:3*
T0*
_class
loc:@memory_size*
use_locking(*
validate_shape(
�
save/Assign_4Assignpolicy/dense/kernelsave/RestoreV2:4*
T0*&
_class
loc:@policy/dense/kernel*
use_locking(*
validate_shape(
�
save/Assign_5Assignpolicy/dense_1/kernelsave/RestoreV2:5*
T0*(
_class
loc:@policy/dense_1/kernel*
use_locking(*
validate_shape(
�
save/Assign_6Assign!policy/main_graph_0/hidden_0/biassave/RestoreV2:6*
T0*4
_class*
(&loc:@policy/main_graph_0/hidden_0/bias*
use_locking(*
validate_shape(
�
save/Assign_7Assign#policy/main_graph_0/hidden_0/kernelsave/RestoreV2:7*
T0*6
_class,
*(loc:@policy/main_graph_0/hidden_0/kernel*
use_locking(*
validate_shape(
�
save/Assign_8Assign!policy/main_graph_0/hidden_1/biassave/RestoreV2:8*
T0*4
_class*
(&loc:@policy/main_graph_0/hidden_1/bias*
use_locking(*
validate_shape(
�
save/Assign_9Assign#policy/main_graph_0/hidden_1/kernelsave/RestoreV2:9*
T0*6
_class,
*(loc:@policy/main_graph_0/hidden_1/kernel*
use_locking(*
validate_shape(
�
save/Assign_10Assigntrainer_major_versionsave/RestoreV2:10*
T0*(
_class
loc:@trainer_major_version*
use_locking(*
validate_shape(
�
save/Assign_11Assigntrainer_minor_versionsave/RestoreV2:11*
T0*(
_class
loc:@trainer_minor_version*
use_locking(*
validate_shape(
�
save/Assign_12Assigntrainer_patch_versionsave/RestoreV2:12*
T0*(
_class
loc:@trainer_patch_version*
use_locking(*
validate_shape(
�
save/Assign_13Assignversion_numbersave/RestoreV2:13*
T0*!
_class
loc:@version_number*
use_locking(*
validate_shape(
�
save/restore_allNoOp^save/Assign^save/Assign_1^save/Assign_10^save/Assign_11^save/Assign_12^save/Assign_13^save/Assign_2^save/Assign_3^save/Assign_4^save/Assign_5^save/Assign_6^save/Assign_7^save/Assign_8^save/Assign_9
�
initNoOp^action_output_shape/Assign^global_step/Assign^is_continuous_control/Assign^memory_size/Assign^policy/dense/kernel/Assign^policy/dense_1/kernel/Assign)^policy/main_graph_0/hidden_0/bias/Assign+^policy/main_graph_0/hidden_0/kernel/Assign)^policy/main_graph_0/hidden_1/bias/Assign+^policy/main_graph_0/hidden_1/kernel/Assign^trainer_major_version/Assign^trainer_minor_version/Assign^trainer_patch_version/Assign^version_number/Assign
e
+optimizer/curiosity_next_vector_observationPlaceholder*
dtype0*
shape:���������
�
Zoptimizer//curiosity_vector_obs_encoder/hidden_0/kernel/Initializer/truncated_normal/shapeConst*J
_class@
><loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/kernel*
dtype0*
valueB"      
�
Yoptimizer//curiosity_vector_obs_encoder/hidden_0/kernel/Initializer/truncated_normal/meanConst*J
_class@
><loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/kernel*
dtype0*
valueB
 *    
�
[optimizer//curiosity_vector_obs_encoder/hidden_0/kernel/Initializer/truncated_normal/stddevConst*J
_class@
><loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/kernel*
dtype0*
valueB
 *���=
�
doptimizer//curiosity_vector_obs_encoder/hidden_0/kernel/Initializer/truncated_normal/TruncatedNormalTruncatedNormalZoptimizer//curiosity_vector_obs_encoder/hidden_0/kernel/Initializer/truncated_normal/shape*
T0*J
_class@
><loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/kernel*
dtype0*
seed�0*
seed2
�
Xoptimizer//curiosity_vector_obs_encoder/hidden_0/kernel/Initializer/truncated_normal/mulMuldoptimizer//curiosity_vector_obs_encoder/hidden_0/kernel/Initializer/truncated_normal/TruncatedNormal[optimizer//curiosity_vector_obs_encoder/hidden_0/kernel/Initializer/truncated_normal/stddev*
T0*J
_class@
><loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/kernel
�
Toptimizer//curiosity_vector_obs_encoder/hidden_0/kernel/Initializer/truncated_normalAddXoptimizer//curiosity_vector_obs_encoder/hidden_0/kernel/Initializer/truncated_normal/mulYoptimizer//curiosity_vector_obs_encoder/hidden_0/kernel/Initializer/truncated_normal/mean*
T0*J
_class@
><loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/kernel
�
7optimizer//curiosity_vector_obs_encoder/hidden_0/kernel
VariableV2*J
_class@
><loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/kernel*
	container *
dtype0*
shape:	�*
shared_name 
�
>optimizer//curiosity_vector_obs_encoder/hidden_0/kernel/AssignAssign7optimizer//curiosity_vector_obs_encoder/hidden_0/kernelToptimizer//curiosity_vector_obs_encoder/hidden_0/kernel/Initializer/truncated_normal*
T0*J
_class@
><loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/kernel*
use_locking(*
validate_shape(
�
<optimizer//curiosity_vector_obs_encoder/hidden_0/kernel/readIdentity7optimizer//curiosity_vector_obs_encoder/hidden_0/kernel*
T0*J
_class@
><loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/kernel
�
Goptimizer//curiosity_vector_obs_encoder/hidden_0/bias/Initializer/zerosConst*H
_class>
<:loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/bias*
dtype0*
valueB�*    
�
5optimizer//curiosity_vector_obs_encoder/hidden_0/bias
VariableV2*H
_class>
<:loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/bias*
	container *
dtype0*
shape:�*
shared_name 
�
<optimizer//curiosity_vector_obs_encoder/hidden_0/bias/AssignAssign5optimizer//curiosity_vector_obs_encoder/hidden_0/biasGoptimizer//curiosity_vector_obs_encoder/hidden_0/bias/Initializer/zeros*
T0*H
_class>
<:loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/bias*
use_locking(*
validate_shape(
�
:optimizer//curiosity_vector_obs_encoder/hidden_0/bias/readIdentity5optimizer//curiosity_vector_obs_encoder/hidden_0/bias*
T0*H
_class>
<:loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/bias
�
6optimizer/curiosity_vector_obs_encoder/hidden_0/MatMulMatMulvector_observation<optimizer//curiosity_vector_obs_encoder/hidden_0/kernel/read*
T0*
transpose_a( *
transpose_b( 
�
7optimizer/curiosity_vector_obs_encoder/hidden_0/BiasAddBiasAdd6optimizer/curiosity_vector_obs_encoder/hidden_0/MatMul:optimizer//curiosity_vector_obs_encoder/hidden_0/bias/read*
T0*
data_formatNHWC
�
7optimizer/curiosity_vector_obs_encoder/hidden_0/SigmoidSigmoid7optimizer/curiosity_vector_obs_encoder/hidden_0/BiasAdd*
T0
�
3optimizer/curiosity_vector_obs_encoder/hidden_0/MulMul7optimizer/curiosity_vector_obs_encoder/hidden_0/BiasAdd7optimizer/curiosity_vector_obs_encoder/hidden_0/Sigmoid*
T0
�
Zoptimizer//curiosity_vector_obs_encoder/hidden_1/kernel/Initializer/truncated_normal/shapeConst*J
_class@
><loc:@optimizer//curiosity_vector_obs_encoder/hidden_1/kernel*
dtype0*
valueB"      
�
Yoptimizer//curiosity_vector_obs_encoder/hidden_1/kernel/Initializer/truncated_normal/meanConst*J
_class@
><loc:@optimizer//curiosity_vector_obs_encoder/hidden_1/kernel*
dtype0*
valueB
 *    
�
[optimizer//curiosity_vector_obs_encoder/hidden_1/kernel/Initializer/truncated_normal/stddevConst*J
_class@
><loc:@optimizer//curiosity_vector_obs_encoder/hidden_1/kernel*
dtype0*
valueB
 *6��=
�
doptimizer//curiosity_vector_obs_encoder/hidden_1/kernel/Initializer/truncated_normal/TruncatedNormalTruncatedNormalZoptimizer//curiosity_vector_obs_encoder/hidden_1/kernel/Initializer/truncated_normal/shape*
T0*J
_class@
><loc:@optimizer//curiosity_vector_obs_encoder/hidden_1/kernel*
dtype0*
seed�0*
seed2
�
Xoptimizer//curiosity_vector_obs_encoder/hidden_1/kernel/Initializer/truncated_normal/mulMuldoptimizer//curiosity_vector_obs_encoder/hidden_1/kernel/Initializer/truncated_normal/TruncatedNormal[optimizer//curiosity_vector_obs_encoder/hidden_1/kernel/Initializer/truncated_normal/stddev*
T0*J
_class@
><loc:@optimizer//curiosity_vector_obs_encoder/hidden_1/kernel
�
Toptimizer//curiosity_vector_obs_encoder/hidden_1/kernel/Initializer/truncated_normalAddXoptimizer//curiosity_vector_obs_encoder/hidden_1/kernel/Initializer/truncated_normal/mulYoptimizer//curiosity_vector_obs_encoder/hidden_1/kernel/Initializer/truncated_normal/mean*
T0*J
_class@
><loc:@optimizer//curiosity_vector_obs_encoder/hidden_1/kernel
�
7optimizer//curiosity_vector_obs_encoder/hidden_1/kernel
VariableV2*J
_class@
><loc:@optimizer//curiosity_vector_obs_encoder/hidden_1/kernel*
	container *
dtype0*
shape:
��*
shared_name 
�
>optimizer//curiosity_vector_obs_encoder/hidden_1/kernel/AssignAssign7optimizer//curiosity_vector_obs_encoder/hidden_1/kernelToptimizer//curiosity_vector_obs_encoder/hidden_1/kernel/Initializer/truncated_normal*
T0*J
_class@
><loc:@optimizer//curiosity_vector_obs_encoder/hidden_1/kernel*
use_locking(*
validate_shape(
�
<optimizer//curiosity_vector_obs_encoder/hidden_1/kernel/readIdentity7optimizer//curiosity_vector_obs_encoder/hidden_1/kernel*
T0*J
_class@
><loc:@optimizer//curiosity_vector_obs_encoder/hidden_1/kernel
�
Goptimizer//curiosity_vector_obs_encoder/hidden_1/bias/Initializer/zerosConst*H
_class>
<:loc:@optimizer//curiosity_vector_obs_encoder/hidden_1/bias*
dtype0*
valueB�*    
�
5optimizer//curiosity_vector_obs_encoder/hidden_1/bias
VariableV2*H
_class>
<:loc:@optimizer//curiosity_vector_obs_encoder/hidden_1/bias*
	container *
dtype0*
shape:�*
shared_name 
�
<optimizer//curiosity_vector_obs_encoder/hidden_1/bias/AssignAssign5optimizer//curiosity_vector_obs_encoder/hidden_1/biasGoptimizer//curiosity_vector_obs_encoder/hidden_1/bias/Initializer/zeros*
T0*H
_class>
<:loc:@optimizer//curiosity_vector_obs_encoder/hidden_1/bias*
use_locking(*
validate_shape(
�
:optimizer//curiosity_vector_obs_encoder/hidden_1/bias/readIdentity5optimizer//curiosity_vector_obs_encoder/hidden_1/bias*
T0*H
_class>
<:loc:@optimizer//curiosity_vector_obs_encoder/hidden_1/bias
�
6optimizer/curiosity_vector_obs_encoder/hidden_1/MatMulMatMul3optimizer/curiosity_vector_obs_encoder/hidden_0/Mul<optimizer//curiosity_vector_obs_encoder/hidden_1/kernel/read*
T0*
transpose_a( *
transpose_b( 
�
7optimizer/curiosity_vector_obs_encoder/hidden_1/BiasAddBiasAdd6optimizer/curiosity_vector_obs_encoder/hidden_1/MatMul:optimizer//curiosity_vector_obs_encoder/hidden_1/bias/read*
T0*
data_formatNHWC
�
7optimizer/curiosity_vector_obs_encoder/hidden_1/SigmoidSigmoid7optimizer/curiosity_vector_obs_encoder/hidden_1/BiasAdd*
T0
�
3optimizer/curiosity_vector_obs_encoder/hidden_1/MulMul7optimizer/curiosity_vector_obs_encoder/hidden_1/BiasAdd7optimizer/curiosity_vector_obs_encoder/hidden_1/Sigmoid*
T0
�
8optimizer/curiosity_vector_obs_encoder_1/hidden_0/MatMulMatMul+optimizer/curiosity_next_vector_observation<optimizer//curiosity_vector_obs_encoder/hidden_0/kernel/read*
T0*
transpose_a( *
transpose_b( 
�
9optimizer/curiosity_vector_obs_encoder_1/hidden_0/BiasAddBiasAdd8optimizer/curiosity_vector_obs_encoder_1/hidden_0/MatMul:optimizer//curiosity_vector_obs_encoder/hidden_0/bias/read*
T0*
data_formatNHWC
�
9optimizer/curiosity_vector_obs_encoder_1/hidden_0/SigmoidSigmoid9optimizer/curiosity_vector_obs_encoder_1/hidden_0/BiasAdd*
T0
�
5optimizer/curiosity_vector_obs_encoder_1/hidden_0/MulMul9optimizer/curiosity_vector_obs_encoder_1/hidden_0/BiasAdd9optimizer/curiosity_vector_obs_encoder_1/hidden_0/Sigmoid*
T0
�
8optimizer/curiosity_vector_obs_encoder_1/hidden_1/MatMulMatMul5optimizer/curiosity_vector_obs_encoder_1/hidden_0/Mul<optimizer//curiosity_vector_obs_encoder/hidden_1/kernel/read*
T0*
transpose_a( *
transpose_b( 
�
9optimizer/curiosity_vector_obs_encoder_1/hidden_1/BiasAddBiasAdd8optimizer/curiosity_vector_obs_encoder_1/hidden_1/MatMul:optimizer//curiosity_vector_obs_encoder/hidden_1/bias/read*
T0*
data_formatNHWC
�
9optimizer/curiosity_vector_obs_encoder_1/hidden_1/SigmoidSigmoid9optimizer/curiosity_vector_obs_encoder_1/hidden_1/BiasAdd*
T0
�
5optimizer/curiosity_vector_obs_encoder_1/hidden_1/MulMul9optimizer/curiosity_vector_obs_encoder_1/hidden_1/BiasAdd9optimizer/curiosity_vector_obs_encoder_1/hidden_1/Sigmoid*
T0
E
optimizer/concat/concat_dimConst*
dtype0*
value	B :
a
optimizer/concat/concatIdentity3optimizer/curiosity_vector_obs_encoder/hidden_1/Mul*
T0
G
optimizer/concat_1/concat_dimConst*
dtype0*
value	B :
e
optimizer/concat_1/concatIdentity5optimizer/curiosity_vector_obs_encoder_1/hidden_1/Mul*
T0
A
optimizer/concat_2/axisConst*
dtype0*
value	B :
�
optimizer/concat_2ConcatV2optimizer/concat/concatoptimizer/concat_1/concatoptimizer/concat_2/axis*
N*
T0*

Tidx0
�
8optimizer//dense/kernel/Initializer/random_uniform/shapeConst**
_class 
loc:@optimizer//dense/kernel*
dtype0*
valueB"      
�
6optimizer//dense/kernel/Initializer/random_uniform/minConst**
_class 
loc:@optimizer//dense/kernel*
dtype0*
valueB
 *���
�
6optimizer//dense/kernel/Initializer/random_uniform/maxConst**
_class 
loc:@optimizer//dense/kernel*
dtype0*
valueB
 *��=
�
@optimizer//dense/kernel/Initializer/random_uniform/RandomUniformRandomUniform8optimizer//dense/kernel/Initializer/random_uniform/shape*
T0**
_class 
loc:@optimizer//dense/kernel*
dtype0*
seed�0*
seed2
�
6optimizer//dense/kernel/Initializer/random_uniform/subSub6optimizer//dense/kernel/Initializer/random_uniform/max6optimizer//dense/kernel/Initializer/random_uniform/min*
T0**
_class 
loc:@optimizer//dense/kernel
�
6optimizer//dense/kernel/Initializer/random_uniform/mulMul@optimizer//dense/kernel/Initializer/random_uniform/RandomUniform6optimizer//dense/kernel/Initializer/random_uniform/sub*
T0**
_class 
loc:@optimizer//dense/kernel
�
2optimizer//dense/kernel/Initializer/random_uniformAdd6optimizer//dense/kernel/Initializer/random_uniform/mul6optimizer//dense/kernel/Initializer/random_uniform/min*
T0**
_class 
loc:@optimizer//dense/kernel
�
optimizer//dense/kernel
VariableV2**
_class 
loc:@optimizer//dense/kernel*
	container *
dtype0*
shape:
��*
shared_name 
�
optimizer//dense/kernel/AssignAssignoptimizer//dense/kernel2optimizer//dense/kernel/Initializer/random_uniform*
T0**
_class 
loc:@optimizer//dense/kernel*
use_locking(*
validate_shape(
v
optimizer//dense/kernel/readIdentityoptimizer//dense/kernel*
T0**
_class 
loc:@optimizer//dense/kernel
�
'optimizer//dense/bias/Initializer/zerosConst*(
_class
loc:@optimizer//dense/bias*
dtype0*
valueB�*    
�
optimizer//dense/bias
VariableV2*(
_class
loc:@optimizer//dense/bias*
	container *
dtype0*
shape:�*
shared_name 
�
optimizer//dense/bias/AssignAssignoptimizer//dense/bias'optimizer//dense/bias/Initializer/zeros*
T0*(
_class
loc:@optimizer//dense/bias*
use_locking(*
validate_shape(
p
optimizer//dense/bias/readIdentityoptimizer//dense/bias*
T0*(
_class
loc:@optimizer//dense/bias
�
optimizer/dense/MatMulMatMuloptimizer/concat_2optimizer//dense/kernel/read*
T0*
transpose_a( *
transpose_b( 
v
optimizer/dense/BiasAddBiasAddoptimizer/dense/MatMuloptimizer//dense/bias/read*
T0*
data_formatNHWC
D
optimizer/dense/SigmoidSigmoidoptimizer/dense/BiasAdd*
T0
U
optimizer/dense/MulMuloptimizer/dense/BiasAddoptimizer/dense/Sigmoid*
T0
�
:optimizer//dense_1/kernel/Initializer/random_uniform/shapeConst*,
_class"
 loc:@optimizer//dense_1/kernel*
dtype0*
valueB"      
�
8optimizer//dense_1/kernel/Initializer/random_uniform/minConst*,
_class"
 loc:@optimizer//dense_1/kernel*
dtype0*
valueB
 *V��
�
8optimizer//dense_1/kernel/Initializer/random_uniform/maxConst*,
_class"
 loc:@optimizer//dense_1/kernel*
dtype0*
valueB
 *V�>
�
Boptimizer//dense_1/kernel/Initializer/random_uniform/RandomUniformRandomUniform:optimizer//dense_1/kernel/Initializer/random_uniform/shape*
T0*,
_class"
 loc:@optimizer//dense_1/kernel*
dtype0*
seed�0*
seed2	
�
8optimizer//dense_1/kernel/Initializer/random_uniform/subSub8optimizer//dense_1/kernel/Initializer/random_uniform/max8optimizer//dense_1/kernel/Initializer/random_uniform/min*
T0*,
_class"
 loc:@optimizer//dense_1/kernel
�
8optimizer//dense_1/kernel/Initializer/random_uniform/mulMulBoptimizer//dense_1/kernel/Initializer/random_uniform/RandomUniform8optimizer//dense_1/kernel/Initializer/random_uniform/sub*
T0*,
_class"
 loc:@optimizer//dense_1/kernel
�
4optimizer//dense_1/kernel/Initializer/random_uniformAdd8optimizer//dense_1/kernel/Initializer/random_uniform/mul8optimizer//dense_1/kernel/Initializer/random_uniform/min*
T0*,
_class"
 loc:@optimizer//dense_1/kernel
�
optimizer//dense_1/kernel
VariableV2*,
_class"
 loc:@optimizer//dense_1/kernel*
	container *
dtype0*
shape:	�*
shared_name 
�
 optimizer//dense_1/kernel/AssignAssignoptimizer//dense_1/kernel4optimizer//dense_1/kernel/Initializer/random_uniform*
T0*,
_class"
 loc:@optimizer//dense_1/kernel*
use_locking(*
validate_shape(
|
optimizer//dense_1/kernel/readIdentityoptimizer//dense_1/kernel*
T0*,
_class"
 loc:@optimizer//dense_1/kernel
�
)optimizer//dense_1/bias/Initializer/zerosConst**
_class 
loc:@optimizer//dense_1/bias*
dtype0*
valueB*    
�
optimizer//dense_1/bias
VariableV2**
_class 
loc:@optimizer//dense_1/bias*
	container *
dtype0*
shape:*
shared_name 
�
optimizer//dense_1/bias/AssignAssignoptimizer//dense_1/bias)optimizer//dense_1/bias/Initializer/zeros*
T0**
_class 
loc:@optimizer//dense_1/bias*
use_locking(*
validate_shape(
v
optimizer//dense_1/bias/readIdentityoptimizer//dense_1/bias*
T0**
_class 
loc:@optimizer//dense_1/bias
�
optimizer/dense_1/MatMulMatMuloptimizer/dense/Muloptimizer//dense_1/kernel/read*
T0*
transpose_a( *
transpose_b( 
|
optimizer/dense_1/BiasAddBiasAddoptimizer/dense_1/MatMuloptimizer//dense_1/bias/read*
T0*
data_formatNHWC
H
optimizer/dense_1/SoftmaxSoftmaxoptimizer/dense_1/BiasAdd*
T0
�
:optimizer//dense_2/kernel/Initializer/random_uniform/shapeConst*,
_class"
 loc:@optimizer//dense_2/kernel*
dtype0*
valueB"      
�
8optimizer//dense_2/kernel/Initializer/random_uniform/minConst*,
_class"
 loc:@optimizer//dense_2/kernel*
dtype0*
valueB
 *V��
�
8optimizer//dense_2/kernel/Initializer/random_uniform/maxConst*,
_class"
 loc:@optimizer//dense_2/kernel*
dtype0*
valueB
 *V�>
�
Boptimizer//dense_2/kernel/Initializer/random_uniform/RandomUniformRandomUniform:optimizer//dense_2/kernel/Initializer/random_uniform/shape*
T0*,
_class"
 loc:@optimizer//dense_2/kernel*
dtype0*
seed�0*
seed2

�
8optimizer//dense_2/kernel/Initializer/random_uniform/subSub8optimizer//dense_2/kernel/Initializer/random_uniform/max8optimizer//dense_2/kernel/Initializer/random_uniform/min*
T0*,
_class"
 loc:@optimizer//dense_2/kernel
�
8optimizer//dense_2/kernel/Initializer/random_uniform/mulMulBoptimizer//dense_2/kernel/Initializer/random_uniform/RandomUniform8optimizer//dense_2/kernel/Initializer/random_uniform/sub*
T0*,
_class"
 loc:@optimizer//dense_2/kernel
�
4optimizer//dense_2/kernel/Initializer/random_uniformAdd8optimizer//dense_2/kernel/Initializer/random_uniform/mul8optimizer//dense_2/kernel/Initializer/random_uniform/min*
T0*,
_class"
 loc:@optimizer//dense_2/kernel
�
optimizer//dense_2/kernel
VariableV2*,
_class"
 loc:@optimizer//dense_2/kernel*
	container *
dtype0*
shape:	�*
shared_name 
�
 optimizer//dense_2/kernel/AssignAssignoptimizer//dense_2/kernel4optimizer//dense_2/kernel/Initializer/random_uniform*
T0*,
_class"
 loc:@optimizer//dense_2/kernel*
use_locking(*
validate_shape(
|
optimizer//dense_2/kernel/readIdentityoptimizer//dense_2/kernel*
T0*,
_class"
 loc:@optimizer//dense_2/kernel
�
)optimizer//dense_2/bias/Initializer/zerosConst**
_class 
loc:@optimizer//dense_2/bias*
dtype0*
valueB*    
�
optimizer//dense_2/bias
VariableV2**
_class 
loc:@optimizer//dense_2/bias*
	container *
dtype0*
shape:*
shared_name 
�
optimizer//dense_2/bias/AssignAssignoptimizer//dense_2/bias)optimizer//dense_2/bias/Initializer/zeros*
T0**
_class 
loc:@optimizer//dense_2/bias*
use_locking(*
validate_shape(
v
optimizer//dense_2/bias/readIdentityoptimizer//dense_2/bias*
T0**
_class 
loc:@optimizer//dense_2/bias
�
optimizer/dense_2/MatMulMatMuloptimizer/dense/Muloptimizer//dense_2/kernel/read*
T0*
transpose_a( *
transpose_b( 
|
optimizer/dense_2/BiasAddBiasAddoptimizer/dense_2/MatMuloptimizer//dense_2/bias/read*
T0*
data_formatNHWC
H
optimizer/dense_2/SoftmaxSoftmaxoptimizer/dense_2/BiasAdd*
T0
A
optimizer/concat_3/axisConst*
dtype0*
value	B :
�
optimizer/concat_3ConcatV2optimizer/dense_1/Softmaxoptimizer/dense_2/Softmaxoptimizer/concat_3/axis*
N*
T0*

Tidx0
<
optimizer/add/yConst*
dtype0*
valueB
 *���.
D
optimizer/addAddV2optimizer/concat_3optimizer/add/y*
T0
,
optimizer/LogLogoptimizer/add*
T0
,
optimizer/NegNegoptimizer/Log*
T0
:
optimizer/mulMuloptimizer/NegStopGradient*
T0
I
optimizer/Sum/reduction_indicesConst*
dtype0*
value	B :
j
optimizer/SumSumoptimizer/muloptimizer/Sum/reduction_indices*
T0*

Tidx0*
	keep_dims( 
b
optimizer/DynamicPartitionDynamicPartitionoptimizer/SumCast*
T0*
num_partitions
=
optimizer/ConstConst*
dtype0*
valueB: 
k
optimizer/MeanMeanoptimizer/DynamicPartition:1optimizer/Const*
T0*

Tidx0*
	keep_dims( 
A
optimizer/concat_4/axisConst*
dtype0*
value	B :
|
optimizer/concat_4ConcatV2optimizer/concat/concatStopGradientoptimizer/concat_4/axis*
N*
T0*

Tidx0
�
:optimizer//dense_3/kernel/Initializer/random_uniform/shapeConst*,
_class"
 loc:@optimizer//dense_3/kernel*
dtype0*
valueB"     
�
8optimizer//dense_3/kernel/Initializer/random_uniform/minConst*,
_class"
 loc:@optimizer//dense_3/kernel*
dtype0*
valueB
 */jܽ
�
8optimizer//dense_3/kernel/Initializer/random_uniform/maxConst*,
_class"
 loc:@optimizer//dense_3/kernel*
dtype0*
valueB
 */j�=
�
Boptimizer//dense_3/kernel/Initializer/random_uniform/RandomUniformRandomUniform:optimizer//dense_3/kernel/Initializer/random_uniform/shape*
T0*,
_class"
 loc:@optimizer//dense_3/kernel*
dtype0*
seed�0*
seed2
�
8optimizer//dense_3/kernel/Initializer/random_uniform/subSub8optimizer//dense_3/kernel/Initializer/random_uniform/max8optimizer//dense_3/kernel/Initializer/random_uniform/min*
T0*,
_class"
 loc:@optimizer//dense_3/kernel
�
8optimizer//dense_3/kernel/Initializer/random_uniform/mulMulBoptimizer//dense_3/kernel/Initializer/random_uniform/RandomUniform8optimizer//dense_3/kernel/Initializer/random_uniform/sub*
T0*,
_class"
 loc:@optimizer//dense_3/kernel
�
4optimizer//dense_3/kernel/Initializer/random_uniformAdd8optimizer//dense_3/kernel/Initializer/random_uniform/mul8optimizer//dense_3/kernel/Initializer/random_uniform/min*
T0*,
_class"
 loc:@optimizer//dense_3/kernel
�
optimizer//dense_3/kernel
VariableV2*,
_class"
 loc:@optimizer//dense_3/kernel*
	container *
dtype0*
shape:
��*
shared_name 
�
 optimizer//dense_3/kernel/AssignAssignoptimizer//dense_3/kernel4optimizer//dense_3/kernel/Initializer/random_uniform*
T0*,
_class"
 loc:@optimizer//dense_3/kernel*
use_locking(*
validate_shape(
|
optimizer//dense_3/kernel/readIdentityoptimizer//dense_3/kernel*
T0*,
_class"
 loc:@optimizer//dense_3/kernel
�
)optimizer//dense_3/bias/Initializer/zerosConst**
_class 
loc:@optimizer//dense_3/bias*
dtype0*
valueB�*    
�
optimizer//dense_3/bias
VariableV2**
_class 
loc:@optimizer//dense_3/bias*
	container *
dtype0*
shape:�*
shared_name 
�
optimizer//dense_3/bias/AssignAssignoptimizer//dense_3/bias)optimizer//dense_3/bias/Initializer/zeros*
T0**
_class 
loc:@optimizer//dense_3/bias*
use_locking(*
validate_shape(
v
optimizer//dense_3/bias/readIdentityoptimizer//dense_3/bias*
T0**
_class 
loc:@optimizer//dense_3/bias
�
optimizer/dense_3/MatMulMatMuloptimizer/concat_4optimizer//dense_3/kernel/read*
T0*
transpose_a( *
transpose_b( 
|
optimizer/dense_3/BiasAddBiasAddoptimizer/dense_3/MatMuloptimizer//dense_3/bias/read*
T0*
data_formatNHWC
H
optimizer/dense_3/SigmoidSigmoidoptimizer/dense_3/BiasAdd*
T0
[
optimizer/dense_3/MulMuloptimizer/dense_3/BiasAddoptimizer/dense_3/Sigmoid*
T0
�
:optimizer//dense_4/kernel/Initializer/random_uniform/shapeConst*,
_class"
 loc:@optimizer//dense_4/kernel*
dtype0*
valueB"      
�
8optimizer//dense_4/kernel/Initializer/random_uniform/minConst*,
_class"
 loc:@optimizer//dense_4/kernel*
dtype0*
valueB
 *׳ݽ
�
8optimizer//dense_4/kernel/Initializer/random_uniform/maxConst*,
_class"
 loc:@optimizer//dense_4/kernel*
dtype0*
valueB
 *׳�=
�
Boptimizer//dense_4/kernel/Initializer/random_uniform/RandomUniformRandomUniform:optimizer//dense_4/kernel/Initializer/random_uniform/shape*
T0*,
_class"
 loc:@optimizer//dense_4/kernel*
dtype0*
seed�0*
seed2
�
8optimizer//dense_4/kernel/Initializer/random_uniform/subSub8optimizer//dense_4/kernel/Initializer/random_uniform/max8optimizer//dense_4/kernel/Initializer/random_uniform/min*
T0*,
_class"
 loc:@optimizer//dense_4/kernel
�
8optimizer//dense_4/kernel/Initializer/random_uniform/mulMulBoptimizer//dense_4/kernel/Initializer/random_uniform/RandomUniform8optimizer//dense_4/kernel/Initializer/random_uniform/sub*
T0*,
_class"
 loc:@optimizer//dense_4/kernel
�
4optimizer//dense_4/kernel/Initializer/random_uniformAdd8optimizer//dense_4/kernel/Initializer/random_uniform/mul8optimizer//dense_4/kernel/Initializer/random_uniform/min*
T0*,
_class"
 loc:@optimizer//dense_4/kernel
�
optimizer//dense_4/kernel
VariableV2*,
_class"
 loc:@optimizer//dense_4/kernel*
	container *
dtype0*
shape:
��*
shared_name 
�
 optimizer//dense_4/kernel/AssignAssignoptimizer//dense_4/kernel4optimizer//dense_4/kernel/Initializer/random_uniform*
T0*,
_class"
 loc:@optimizer//dense_4/kernel*
use_locking(*
validate_shape(
|
optimizer//dense_4/kernel/readIdentityoptimizer//dense_4/kernel*
T0*,
_class"
 loc:@optimizer//dense_4/kernel
�
)optimizer//dense_4/bias/Initializer/zerosConst**
_class 
loc:@optimizer//dense_4/bias*
dtype0*
valueB�*    
�
optimizer//dense_4/bias
VariableV2**
_class 
loc:@optimizer//dense_4/bias*
	container *
dtype0*
shape:�*
shared_name 
�
optimizer//dense_4/bias/AssignAssignoptimizer//dense_4/bias)optimizer//dense_4/bias/Initializer/zeros*
T0**
_class 
loc:@optimizer//dense_4/bias*
use_locking(*
validate_shape(
v
optimizer//dense_4/bias/readIdentityoptimizer//dense_4/bias*
T0**
_class 
loc:@optimizer//dense_4/bias
�
optimizer/dense_4/MatMulMatMuloptimizer/dense_3/Muloptimizer//dense_4/kernel/read*
T0*
transpose_a( *
transpose_b( 
|
optimizer/dense_4/BiasAddBiasAddoptimizer/dense_4/MatMuloptimizer//dense_4/bias/read*
T0*
data_formatNHWC
o
optimizer/SquaredDifferenceSquaredDifferenceoptimizer/dense_4/BiasAddoptimizer/concat_1/concat*
T0
K
!optimizer/Sum_1/reduction_indicesConst*
dtype0*
value	B :
|
optimizer/Sum_1Sumoptimizer/SquaredDifference!optimizer/Sum_1/reduction_indices*
T0*

Tidx0*
	keep_dims( 
>
optimizer/mul_1/xConst*
dtype0*
valueB
 *   ?
C
optimizer/mul_1Muloptimizer/mul_1/xoptimizer/Sum_1*
T0
f
optimizer/DynamicPartition_1DynamicPartitionoptimizer/mul_1Cast*
T0*
num_partitions
?
optimizer/Const_1Const*
dtype0*
valueB: 
q
optimizer/Mean_1Meanoptimizer/DynamicPartition_1:1optimizer/Const_1*
T0*

Tidx0*
	keep_dims( 
>
optimizer/mul_2/xConst*
dtype0*
valueB
 *��L>
D
optimizer/mul_2Muloptimizer/mul_2/xoptimizer/Mean_1*
T0
>
optimizer/mul_3/xConst*
dtype0*
valueB
 *��L?
B
optimizer/mul_3Muloptimizer/mul_3/xoptimizer/Mean*
T0
C
optimizer/add_1AddV2optimizer/mul_2optimizer/mul_3*
T0
>
optimizer/mul_4/xConst*
dtype0*
valueB
 *   A
C
optimizer/mul_4Muloptimizer/mul_4/xoptimizer/add_1*
T0
B
optimizer/gradients/ShapeConst*
dtype0*
valueB 
J
optimizer/gradients/grad_ys_0Const*
dtype0*
valueB
 *  �?
u
optimizer/gradients/FillFilloptimizer/gradients/Shapeoptimizer/gradients/grad_ys_0*
T0*

index_type0
g
,optimizer/gradients/optimizer/mul_4_grad/MulMuloptimizer/gradients/Filloptimizer/add_1*
T0
k
.optimizer/gradients/optimizer/mul_4_grad/Mul_1Muloptimizer/gradients/Filloptimizer/mul_4/x*
T0
�
9optimizer/gradients/optimizer/mul_4_grad/tuple/group_depsNoOp-^optimizer/gradients/optimizer/mul_4_grad/Mul/^optimizer/gradients/optimizer/mul_4_grad/Mul_1
�
Aoptimizer/gradients/optimizer/mul_4_grad/tuple/control_dependencyIdentity,optimizer/gradients/optimizer/mul_4_grad/Mul:^optimizer/gradients/optimizer/mul_4_grad/tuple/group_deps*
T0*?
_class5
31loc:@optimizer/gradients/optimizer/mul_4_grad/Mul
�
Coptimizer/gradients/optimizer/mul_4_grad/tuple/control_dependency_1Identity.optimizer/gradients/optimizer/mul_4_grad/Mul_1:^optimizer/gradients/optimizer/mul_4_grad/tuple/group_deps*
T0*A
_class7
53loc:@optimizer/gradients/optimizer/mul_4_grad/Mul_1
�
9optimizer/gradients/optimizer/add_1_grad/tuple/group_depsNoOpD^optimizer/gradients/optimizer/mul_4_grad/tuple/control_dependency_1
�
Aoptimizer/gradients/optimizer/add_1_grad/tuple/control_dependencyIdentityCoptimizer/gradients/optimizer/mul_4_grad/tuple/control_dependency_1:^optimizer/gradients/optimizer/add_1_grad/tuple/group_deps*
T0*A
_class7
53loc:@optimizer/gradients/optimizer/mul_4_grad/Mul_1
�
Coptimizer/gradients/optimizer/add_1_grad/tuple/control_dependency_1IdentityCoptimizer/gradients/optimizer/mul_4_grad/tuple/control_dependency_1:^optimizer/gradients/optimizer/add_1_grad/tuple/group_deps*
T0*A
_class7
53loc:@optimizer/gradients/optimizer/mul_4_grad/Mul_1
�
,optimizer/gradients/optimizer/mul_2_grad/MulMulAoptimizer/gradients/optimizer/add_1_grad/tuple/control_dependencyoptimizer/Mean_1*
T0
�
.optimizer/gradients/optimizer/mul_2_grad/Mul_1MulAoptimizer/gradients/optimizer/add_1_grad/tuple/control_dependencyoptimizer/mul_2/x*
T0
�
9optimizer/gradients/optimizer/mul_2_grad/tuple/group_depsNoOp-^optimizer/gradients/optimizer/mul_2_grad/Mul/^optimizer/gradients/optimizer/mul_2_grad/Mul_1
�
Aoptimizer/gradients/optimizer/mul_2_grad/tuple/control_dependencyIdentity,optimizer/gradients/optimizer/mul_2_grad/Mul:^optimizer/gradients/optimizer/mul_2_grad/tuple/group_deps*
T0*?
_class5
31loc:@optimizer/gradients/optimizer/mul_2_grad/Mul
�
Coptimizer/gradients/optimizer/mul_2_grad/tuple/control_dependency_1Identity.optimizer/gradients/optimizer/mul_2_grad/Mul_1:^optimizer/gradients/optimizer/mul_2_grad/tuple/group_deps*
T0*A
_class7
53loc:@optimizer/gradients/optimizer/mul_2_grad/Mul_1
�
,optimizer/gradients/optimizer/mul_3_grad/MulMulCoptimizer/gradients/optimizer/add_1_grad/tuple/control_dependency_1optimizer/Mean*
T0
�
.optimizer/gradients/optimizer/mul_3_grad/Mul_1MulCoptimizer/gradients/optimizer/add_1_grad/tuple/control_dependency_1optimizer/mul_3/x*
T0
�
9optimizer/gradients/optimizer/mul_3_grad/tuple/group_depsNoOp-^optimizer/gradients/optimizer/mul_3_grad/Mul/^optimizer/gradients/optimizer/mul_3_grad/Mul_1
�
Aoptimizer/gradients/optimizer/mul_3_grad/tuple/control_dependencyIdentity,optimizer/gradients/optimizer/mul_3_grad/Mul:^optimizer/gradients/optimizer/mul_3_grad/tuple/group_deps*
T0*?
_class5
31loc:@optimizer/gradients/optimizer/mul_3_grad/Mul
�
Coptimizer/gradients/optimizer/mul_3_grad/tuple/control_dependency_1Identity.optimizer/gradients/optimizer/mul_3_grad/Mul_1:^optimizer/gradients/optimizer/mul_3_grad/tuple/group_deps*
T0*A
_class7
53loc:@optimizer/gradients/optimizer/mul_3_grad/Mul_1
e
7optimizer/gradients/optimizer/Mean_1_grad/Reshape/shapeConst*
dtype0*
valueB:
�
1optimizer/gradients/optimizer/Mean_1_grad/ReshapeReshapeCoptimizer/gradients/optimizer/mul_2_grad/tuple/control_dependency_17optimizer/gradients/optimizer/Mean_1_grad/Reshape/shape*
T0*
Tshape0
q
/optimizer/gradients/optimizer/Mean_1_grad/ShapeShapeoptimizer/DynamicPartition_1:1*
T0*
out_type0
�
.optimizer/gradients/optimizer/Mean_1_grad/TileTile1optimizer/gradients/optimizer/Mean_1_grad/Reshape/optimizer/gradients/optimizer/Mean_1_grad/Shape*
T0*

Tmultiples0
s
1optimizer/gradients/optimizer/Mean_1_grad/Shape_1Shapeoptimizer/DynamicPartition_1:1*
T0*
out_type0
Z
1optimizer/gradients/optimizer/Mean_1_grad/Shape_2Const*
dtype0*
valueB 
]
/optimizer/gradients/optimizer/Mean_1_grad/ConstConst*
dtype0*
valueB: 
�
.optimizer/gradients/optimizer/Mean_1_grad/ProdProd1optimizer/gradients/optimizer/Mean_1_grad/Shape_1/optimizer/gradients/optimizer/Mean_1_grad/Const*
T0*

Tidx0*
	keep_dims( 
_
1optimizer/gradients/optimizer/Mean_1_grad/Const_1Const*
dtype0*
valueB: 
�
0optimizer/gradients/optimizer/Mean_1_grad/Prod_1Prod1optimizer/gradients/optimizer/Mean_1_grad/Shape_21optimizer/gradients/optimizer/Mean_1_grad/Const_1*
T0*

Tidx0*
	keep_dims( 
]
3optimizer/gradients/optimizer/Mean_1_grad/Maximum/yConst*
dtype0*
value	B :
�
1optimizer/gradients/optimizer/Mean_1_grad/MaximumMaximum0optimizer/gradients/optimizer/Mean_1_grad/Prod_13optimizer/gradients/optimizer/Mean_1_grad/Maximum/y*
T0
�
2optimizer/gradients/optimizer/Mean_1_grad/floordivFloorDiv.optimizer/gradients/optimizer/Mean_1_grad/Prod1optimizer/gradients/optimizer/Mean_1_grad/Maximum*
T0
�
.optimizer/gradients/optimizer/Mean_1_grad/CastCast2optimizer/gradients/optimizer/Mean_1_grad/floordiv*

DstT0*

SrcT0*
Truncate( 
�
1optimizer/gradients/optimizer/Mean_1_grad/truedivRealDiv.optimizer/gradients/optimizer/Mean_1_grad/Tile.optimizer/gradients/optimizer/Mean_1_grad/Cast*
T0
c
5optimizer/gradients/optimizer/Mean_grad/Reshape/shapeConst*
dtype0*
valueB:
�
/optimizer/gradients/optimizer/Mean_grad/ReshapeReshapeCoptimizer/gradients/optimizer/mul_3_grad/tuple/control_dependency_15optimizer/gradients/optimizer/Mean_grad/Reshape/shape*
T0*
Tshape0
m
-optimizer/gradients/optimizer/Mean_grad/ShapeShapeoptimizer/DynamicPartition:1*
T0*
out_type0
�
,optimizer/gradients/optimizer/Mean_grad/TileTile/optimizer/gradients/optimizer/Mean_grad/Reshape-optimizer/gradients/optimizer/Mean_grad/Shape*
T0*

Tmultiples0
o
/optimizer/gradients/optimizer/Mean_grad/Shape_1Shapeoptimizer/DynamicPartition:1*
T0*
out_type0
X
/optimizer/gradients/optimizer/Mean_grad/Shape_2Const*
dtype0*
valueB 
[
-optimizer/gradients/optimizer/Mean_grad/ConstConst*
dtype0*
valueB: 
�
,optimizer/gradients/optimizer/Mean_grad/ProdProd/optimizer/gradients/optimizer/Mean_grad/Shape_1-optimizer/gradients/optimizer/Mean_grad/Const*
T0*

Tidx0*
	keep_dims( 
]
/optimizer/gradients/optimizer/Mean_grad/Const_1Const*
dtype0*
valueB: 
�
.optimizer/gradients/optimizer/Mean_grad/Prod_1Prod/optimizer/gradients/optimizer/Mean_grad/Shape_2/optimizer/gradients/optimizer/Mean_grad/Const_1*
T0*

Tidx0*
	keep_dims( 
[
1optimizer/gradients/optimizer/Mean_grad/Maximum/yConst*
dtype0*
value	B :
�
/optimizer/gradients/optimizer/Mean_grad/MaximumMaximum.optimizer/gradients/optimizer/Mean_grad/Prod_11optimizer/gradients/optimizer/Mean_grad/Maximum/y*
T0
�
0optimizer/gradients/optimizer/Mean_grad/floordivFloorDiv,optimizer/gradients/optimizer/Mean_grad/Prod/optimizer/gradients/optimizer/Mean_grad/Maximum*
T0
�
,optimizer/gradients/optimizer/Mean_grad/CastCast0optimizer/gradients/optimizer/Mean_grad/floordiv*

DstT0*

SrcT0*
Truncate( 
�
/optimizer/gradients/optimizer/Mean_grad/truedivRealDiv,optimizer/gradients/optimizer/Mean_grad/Tile,optimizer/gradients/optimizer/Mean_grad/Cast*
T0
R
optimizer/gradients/zeros_like	ZerosLikeoptimizer/DynamicPartition_1*
T0
c
;optimizer/gradients/optimizer/DynamicPartition_1_grad/ShapeShapeCast*
T0*
out_type0
i
;optimizer/gradients/optimizer/DynamicPartition_1_grad/ConstConst*
dtype0*
valueB: 
�
:optimizer/gradients/optimizer/DynamicPartition_1_grad/ProdProd;optimizer/gradients/optimizer/DynamicPartition_1_grad/Shape;optimizer/gradients/optimizer/DynamicPartition_1_grad/Const*
T0*

Tidx0*
	keep_dims( 
k
Aoptimizer/gradients/optimizer/DynamicPartition_1_grad/range/startConst*
dtype0*
value	B : 
k
Aoptimizer/gradients/optimizer/DynamicPartition_1_grad/range/deltaConst*
dtype0*
value	B :
�
;optimizer/gradients/optimizer/DynamicPartition_1_grad/rangeRangeAoptimizer/gradients/optimizer/DynamicPartition_1_grad/range/start:optimizer/gradients/optimizer/DynamicPartition_1_grad/ProdAoptimizer/gradients/optimizer/DynamicPartition_1_grad/range/delta*

Tidx0
�
=optimizer/gradients/optimizer/DynamicPartition_1_grad/ReshapeReshape;optimizer/gradients/optimizer/DynamicPartition_1_grad/range;optimizer/gradients/optimizer/DynamicPartition_1_grad/Shape*
T0*
Tshape0
�
Foptimizer/gradients/optimizer/DynamicPartition_1_grad/DynamicPartitionDynamicPartition=optimizer/gradients/optimizer/DynamicPartition_1_grad/ReshapeCast*
T0*
num_partitions
�
Koptimizer/gradients/optimizer/DynamicPartition_1_grad/ParallelDynamicStitchParallelDynamicStitchFoptimizer/gradients/optimizer/DynamicPartition_1_grad/DynamicPartitionHoptimizer/gradients/optimizer/DynamicPartition_1_grad/DynamicPartition:1optimizer/gradients/zeros_like1optimizer/gradients/optimizer/Mean_1_grad/truediv*
N*
T0
p
=optimizer/gradients/optimizer/DynamicPartition_1_grad/Shape_1Shapeoptimizer/mul_1*
T0*
out_type0
�
?optimizer/gradients/optimizer/DynamicPartition_1_grad/Reshape_1ReshapeKoptimizer/gradients/optimizer/DynamicPartition_1_grad/ParallelDynamicStitch=optimizer/gradients/optimizer/DynamicPartition_1_grad/Shape_1*
T0*
Tshape0
R
 optimizer/gradients/zeros_like_1	ZerosLikeoptimizer/DynamicPartition*
T0
a
9optimizer/gradients/optimizer/DynamicPartition_grad/ShapeShapeCast*
T0*
out_type0
g
9optimizer/gradients/optimizer/DynamicPartition_grad/ConstConst*
dtype0*
valueB: 
�
8optimizer/gradients/optimizer/DynamicPartition_grad/ProdProd9optimizer/gradients/optimizer/DynamicPartition_grad/Shape9optimizer/gradients/optimizer/DynamicPartition_grad/Const*
T0*

Tidx0*
	keep_dims( 
i
?optimizer/gradients/optimizer/DynamicPartition_grad/range/startConst*
dtype0*
value	B : 
i
?optimizer/gradients/optimizer/DynamicPartition_grad/range/deltaConst*
dtype0*
value	B :
�
9optimizer/gradients/optimizer/DynamicPartition_grad/rangeRange?optimizer/gradients/optimizer/DynamicPartition_grad/range/start8optimizer/gradients/optimizer/DynamicPartition_grad/Prod?optimizer/gradients/optimizer/DynamicPartition_grad/range/delta*

Tidx0
�
;optimizer/gradients/optimizer/DynamicPartition_grad/ReshapeReshape9optimizer/gradients/optimizer/DynamicPartition_grad/range9optimizer/gradients/optimizer/DynamicPartition_grad/Shape*
T0*
Tshape0
�
Doptimizer/gradients/optimizer/DynamicPartition_grad/DynamicPartitionDynamicPartition;optimizer/gradients/optimizer/DynamicPartition_grad/ReshapeCast*
T0*
num_partitions
�
Ioptimizer/gradients/optimizer/DynamicPartition_grad/ParallelDynamicStitchParallelDynamicStitchDoptimizer/gradients/optimizer/DynamicPartition_grad/DynamicPartitionFoptimizer/gradients/optimizer/DynamicPartition_grad/DynamicPartition:1 optimizer/gradients/zeros_like_1/optimizer/gradients/optimizer/Mean_grad/truediv*
N*
T0
l
;optimizer/gradients/optimizer/DynamicPartition_grad/Shape_1Shapeoptimizer/Sum*
T0*
out_type0
�
=optimizer/gradients/optimizer/DynamicPartition_grad/Reshape_1ReshapeIoptimizer/gradients/optimizer/DynamicPartition_grad/ParallelDynamicStitch;optimizer/gradients/optimizer/DynamicPartition_grad/Shape_1*
T0*
Tshape0
c
.optimizer/gradients/optimizer/mul_1_grad/ShapeShapeoptimizer/mul_1/x*
T0*
out_type0
c
0optimizer/gradients/optimizer/mul_1_grad/Shape_1Shapeoptimizer/Sum_1*
T0*
out_type0
�
>optimizer/gradients/optimizer/mul_1_grad/BroadcastGradientArgsBroadcastGradientArgs.optimizer/gradients/optimizer/mul_1_grad/Shape0optimizer/gradients/optimizer/mul_1_grad/Shape_1*
T0
�
,optimizer/gradients/optimizer/mul_1_grad/MulMul?optimizer/gradients/optimizer/DynamicPartition_1_grad/Reshape_1optimizer/Sum_1*
T0
�
,optimizer/gradients/optimizer/mul_1_grad/SumSum,optimizer/gradients/optimizer/mul_1_grad/Mul>optimizer/gradients/optimizer/mul_1_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
0optimizer/gradients/optimizer/mul_1_grad/ReshapeReshape,optimizer/gradients/optimizer/mul_1_grad/Sum.optimizer/gradients/optimizer/mul_1_grad/Shape*
T0*
Tshape0
�
.optimizer/gradients/optimizer/mul_1_grad/Mul_1Muloptimizer/mul_1/x?optimizer/gradients/optimizer/DynamicPartition_1_grad/Reshape_1*
T0
�
.optimizer/gradients/optimizer/mul_1_grad/Sum_1Sum.optimizer/gradients/optimizer/mul_1_grad/Mul_1@optimizer/gradients/optimizer/mul_1_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
2optimizer/gradients/optimizer/mul_1_grad/Reshape_1Reshape.optimizer/gradients/optimizer/mul_1_grad/Sum_10optimizer/gradients/optimizer/mul_1_grad/Shape_1*
T0*
Tshape0
�
9optimizer/gradients/optimizer/mul_1_grad/tuple/group_depsNoOp1^optimizer/gradients/optimizer/mul_1_grad/Reshape3^optimizer/gradients/optimizer/mul_1_grad/Reshape_1
�
Aoptimizer/gradients/optimizer/mul_1_grad/tuple/control_dependencyIdentity0optimizer/gradients/optimizer/mul_1_grad/Reshape:^optimizer/gradients/optimizer/mul_1_grad/tuple/group_deps*
T0*C
_class9
75loc:@optimizer/gradients/optimizer/mul_1_grad/Reshape
�
Coptimizer/gradients/optimizer/mul_1_grad/tuple/control_dependency_1Identity2optimizer/gradients/optimizer/mul_1_grad/Reshape_1:^optimizer/gradients/optimizer/mul_1_grad/tuple/group_deps*
T0*E
_class;
97loc:@optimizer/gradients/optimizer/mul_1_grad/Reshape_1
]
,optimizer/gradients/optimizer/Sum_grad/ShapeShapeoptimizer/mul*
T0*
out_type0
�
+optimizer/gradients/optimizer/Sum_grad/SizeConst*?
_class5
31loc:@optimizer/gradients/optimizer/Sum_grad/Shape*
dtype0*
value	B :
�
*optimizer/gradients/optimizer/Sum_grad/addAddV2optimizer/Sum/reduction_indices+optimizer/gradients/optimizer/Sum_grad/Size*
T0*?
_class5
31loc:@optimizer/gradients/optimizer/Sum_grad/Shape
�
*optimizer/gradients/optimizer/Sum_grad/modFloorMod*optimizer/gradients/optimizer/Sum_grad/add+optimizer/gradients/optimizer/Sum_grad/Size*
T0*?
_class5
31loc:@optimizer/gradients/optimizer/Sum_grad/Shape
�
.optimizer/gradients/optimizer/Sum_grad/Shape_1Const*?
_class5
31loc:@optimizer/gradients/optimizer/Sum_grad/Shape*
dtype0*
valueB 
�
2optimizer/gradients/optimizer/Sum_grad/range/startConst*?
_class5
31loc:@optimizer/gradients/optimizer/Sum_grad/Shape*
dtype0*
value	B : 
�
2optimizer/gradients/optimizer/Sum_grad/range/deltaConst*?
_class5
31loc:@optimizer/gradients/optimizer/Sum_grad/Shape*
dtype0*
value	B :
�
,optimizer/gradients/optimizer/Sum_grad/rangeRange2optimizer/gradients/optimizer/Sum_grad/range/start+optimizer/gradients/optimizer/Sum_grad/Size2optimizer/gradients/optimizer/Sum_grad/range/delta*

Tidx0*?
_class5
31loc:@optimizer/gradients/optimizer/Sum_grad/Shape
�
1optimizer/gradients/optimizer/Sum_grad/Fill/valueConst*?
_class5
31loc:@optimizer/gradients/optimizer/Sum_grad/Shape*
dtype0*
value	B :
�
+optimizer/gradients/optimizer/Sum_grad/FillFill.optimizer/gradients/optimizer/Sum_grad/Shape_11optimizer/gradients/optimizer/Sum_grad/Fill/value*
T0*?
_class5
31loc:@optimizer/gradients/optimizer/Sum_grad/Shape*

index_type0
�
4optimizer/gradients/optimizer/Sum_grad/DynamicStitchDynamicStitch,optimizer/gradients/optimizer/Sum_grad/range*optimizer/gradients/optimizer/Sum_grad/mod,optimizer/gradients/optimizer/Sum_grad/Shape+optimizer/gradients/optimizer/Sum_grad/Fill*
N*
T0*?
_class5
31loc:@optimizer/gradients/optimizer/Sum_grad/Shape
�
.optimizer/gradients/optimizer/Sum_grad/ReshapeReshape=optimizer/gradients/optimizer/DynamicPartition_grad/Reshape_14optimizer/gradients/optimizer/Sum_grad/DynamicStitch*
T0*
Tshape0
�
2optimizer/gradients/optimizer/Sum_grad/BroadcastToBroadcastTo.optimizer/gradients/optimizer/Sum_grad/Reshape,optimizer/gradients/optimizer/Sum_grad/Shape*
T0*

Tidx0
m
.optimizer/gradients/optimizer/Sum_1_grad/ShapeShapeoptimizer/SquaredDifference*
T0*
out_type0
�
-optimizer/gradients/optimizer/Sum_1_grad/SizeConst*A
_class7
53loc:@optimizer/gradients/optimizer/Sum_1_grad/Shape*
dtype0*
value	B :
�
,optimizer/gradients/optimizer/Sum_1_grad/addAddV2!optimizer/Sum_1/reduction_indices-optimizer/gradients/optimizer/Sum_1_grad/Size*
T0*A
_class7
53loc:@optimizer/gradients/optimizer/Sum_1_grad/Shape
�
,optimizer/gradients/optimizer/Sum_1_grad/modFloorMod,optimizer/gradients/optimizer/Sum_1_grad/add-optimizer/gradients/optimizer/Sum_1_grad/Size*
T0*A
_class7
53loc:@optimizer/gradients/optimizer/Sum_1_grad/Shape
�
0optimizer/gradients/optimizer/Sum_1_grad/Shape_1Const*A
_class7
53loc:@optimizer/gradients/optimizer/Sum_1_grad/Shape*
dtype0*
valueB 
�
4optimizer/gradients/optimizer/Sum_1_grad/range/startConst*A
_class7
53loc:@optimizer/gradients/optimizer/Sum_1_grad/Shape*
dtype0*
value	B : 
�
4optimizer/gradients/optimizer/Sum_1_grad/range/deltaConst*A
_class7
53loc:@optimizer/gradients/optimizer/Sum_1_grad/Shape*
dtype0*
value	B :
�
.optimizer/gradients/optimizer/Sum_1_grad/rangeRange4optimizer/gradients/optimizer/Sum_1_grad/range/start-optimizer/gradients/optimizer/Sum_1_grad/Size4optimizer/gradients/optimizer/Sum_1_grad/range/delta*

Tidx0*A
_class7
53loc:@optimizer/gradients/optimizer/Sum_1_grad/Shape
�
3optimizer/gradients/optimizer/Sum_1_grad/Fill/valueConst*A
_class7
53loc:@optimizer/gradients/optimizer/Sum_1_grad/Shape*
dtype0*
value	B :
�
-optimizer/gradients/optimizer/Sum_1_grad/FillFill0optimizer/gradients/optimizer/Sum_1_grad/Shape_13optimizer/gradients/optimizer/Sum_1_grad/Fill/value*
T0*A
_class7
53loc:@optimizer/gradients/optimizer/Sum_1_grad/Shape*

index_type0
�
6optimizer/gradients/optimizer/Sum_1_grad/DynamicStitchDynamicStitch.optimizer/gradients/optimizer/Sum_1_grad/range,optimizer/gradients/optimizer/Sum_1_grad/mod.optimizer/gradients/optimizer/Sum_1_grad/Shape-optimizer/gradients/optimizer/Sum_1_grad/Fill*
N*
T0*A
_class7
53loc:@optimizer/gradients/optimizer/Sum_1_grad/Shape
�
0optimizer/gradients/optimizer/Sum_1_grad/ReshapeReshapeCoptimizer/gradients/optimizer/mul_1_grad/tuple/control_dependency_16optimizer/gradients/optimizer/Sum_1_grad/DynamicStitch*
T0*
Tshape0
�
4optimizer/gradients/optimizer/Sum_1_grad/BroadcastToBroadcastTo0optimizer/gradients/optimizer/Sum_1_grad/Reshape.optimizer/gradients/optimizer/Sum_1_grad/Shape*
T0*

Tidx0
]
,optimizer/gradients/optimizer/mul_grad/ShapeShapeoptimizer/Neg*
T0*
out_type0
^
.optimizer/gradients/optimizer/mul_grad/Shape_1ShapeStopGradient*
T0*
out_type0
�
<optimizer/gradients/optimizer/mul_grad/BroadcastGradientArgsBroadcastGradientArgs,optimizer/gradients/optimizer/mul_grad/Shape.optimizer/gradients/optimizer/mul_grad/Shape_1*
T0
|
*optimizer/gradients/optimizer/mul_grad/MulMul2optimizer/gradients/optimizer/Sum_grad/BroadcastToStopGradient*
T0
�
*optimizer/gradients/optimizer/mul_grad/SumSum*optimizer/gradients/optimizer/mul_grad/Mul<optimizer/gradients/optimizer/mul_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
.optimizer/gradients/optimizer/mul_grad/ReshapeReshape*optimizer/gradients/optimizer/mul_grad/Sum,optimizer/gradients/optimizer/mul_grad/Shape*
T0*
Tshape0

,optimizer/gradients/optimizer/mul_grad/Mul_1Muloptimizer/Neg2optimizer/gradients/optimizer/Sum_grad/BroadcastTo*
T0
�
,optimizer/gradients/optimizer/mul_grad/Sum_1Sum,optimizer/gradients/optimizer/mul_grad/Mul_1>optimizer/gradients/optimizer/mul_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
0optimizer/gradients/optimizer/mul_grad/Reshape_1Reshape,optimizer/gradients/optimizer/mul_grad/Sum_1.optimizer/gradients/optimizer/mul_grad/Shape_1*
T0*
Tshape0
�
7optimizer/gradients/optimizer/mul_grad/tuple/group_depsNoOp/^optimizer/gradients/optimizer/mul_grad/Reshape1^optimizer/gradients/optimizer/mul_grad/Reshape_1
�
?optimizer/gradients/optimizer/mul_grad/tuple/control_dependencyIdentity.optimizer/gradients/optimizer/mul_grad/Reshape8^optimizer/gradients/optimizer/mul_grad/tuple/group_deps*
T0*A
_class7
53loc:@optimizer/gradients/optimizer/mul_grad/Reshape
�
Aoptimizer/gradients/optimizer/mul_grad/tuple/control_dependency_1Identity0optimizer/gradients/optimizer/mul_grad/Reshape_18^optimizer/gradients/optimizer/mul_grad/tuple/group_deps*
T0*C
_class9
75loc:@optimizer/gradients/optimizer/mul_grad/Reshape_1
�
;optimizer/gradients/optimizer/SquaredDifference_grad/scalarConst5^optimizer/gradients/optimizer/Sum_1_grad/BroadcastTo*
dtype0*
valueB
 *   @
�
8optimizer/gradients/optimizer/SquaredDifference_grad/MulMul;optimizer/gradients/optimizer/SquaredDifference_grad/scalar4optimizer/gradients/optimizer/Sum_1_grad/BroadcastTo*
T0
�
8optimizer/gradients/optimizer/SquaredDifference_grad/subSuboptimizer/dense_4/BiasAddoptimizer/concat_1/concat5^optimizer/gradients/optimizer/Sum_1_grad/BroadcastTo*
T0
�
:optimizer/gradients/optimizer/SquaredDifference_grad/mul_1Mul8optimizer/gradients/optimizer/SquaredDifference_grad/Mul8optimizer/gradients/optimizer/SquaredDifference_grad/sub*
T0
w
:optimizer/gradients/optimizer/SquaredDifference_grad/ShapeShapeoptimizer/dense_4/BiasAdd*
T0*
out_type0
y
<optimizer/gradients/optimizer/SquaredDifference_grad/Shape_1Shapeoptimizer/concat_1/concat*
T0*
out_type0
�
Joptimizer/gradients/optimizer/SquaredDifference_grad/BroadcastGradientArgsBroadcastGradientArgs:optimizer/gradients/optimizer/SquaredDifference_grad/Shape<optimizer/gradients/optimizer/SquaredDifference_grad/Shape_1*
T0
�
8optimizer/gradients/optimizer/SquaredDifference_grad/SumSum:optimizer/gradients/optimizer/SquaredDifference_grad/mul_1Joptimizer/gradients/optimizer/SquaredDifference_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
<optimizer/gradients/optimizer/SquaredDifference_grad/ReshapeReshape8optimizer/gradients/optimizer/SquaredDifference_grad/Sum:optimizer/gradients/optimizer/SquaredDifference_grad/Shape*
T0*
Tshape0
�
:optimizer/gradients/optimizer/SquaredDifference_grad/Sum_1Sum:optimizer/gradients/optimizer/SquaredDifference_grad/mul_1Loptimizer/gradients/optimizer/SquaredDifference_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
>optimizer/gradients/optimizer/SquaredDifference_grad/Reshape_1Reshape:optimizer/gradients/optimizer/SquaredDifference_grad/Sum_1<optimizer/gradients/optimizer/SquaredDifference_grad/Shape_1*
T0*
Tshape0
�
8optimizer/gradients/optimizer/SquaredDifference_grad/NegNeg>optimizer/gradients/optimizer/SquaredDifference_grad/Reshape_1*
T0
�
Eoptimizer/gradients/optimizer/SquaredDifference_grad/tuple/group_depsNoOp9^optimizer/gradients/optimizer/SquaredDifference_grad/Neg=^optimizer/gradients/optimizer/SquaredDifference_grad/Reshape
�
Moptimizer/gradients/optimizer/SquaredDifference_grad/tuple/control_dependencyIdentity<optimizer/gradients/optimizer/SquaredDifference_grad/ReshapeF^optimizer/gradients/optimizer/SquaredDifference_grad/tuple/group_deps*
T0*O
_classE
CAloc:@optimizer/gradients/optimizer/SquaredDifference_grad/Reshape
�
Ooptimizer/gradients/optimizer/SquaredDifference_grad/tuple/control_dependency_1Identity8optimizer/gradients/optimizer/SquaredDifference_grad/NegF^optimizer/gradients/optimizer/SquaredDifference_grad/tuple/group_deps*
T0*K
_classA
?=loc:@optimizer/gradients/optimizer/SquaredDifference_grad/Neg
{
*optimizer/gradients/optimizer/Neg_grad/NegNeg?optimizer/gradients/optimizer/mul_grad/tuple/control_dependency*
T0
�
>optimizer/gradients/optimizer/dense_4/BiasAdd_grad/BiasAddGradBiasAddGradMoptimizer/gradients/optimizer/SquaredDifference_grad/tuple/control_dependency*
T0*
data_formatNHWC
�
Coptimizer/gradients/optimizer/dense_4/BiasAdd_grad/tuple/group_depsNoOpN^optimizer/gradients/optimizer/SquaredDifference_grad/tuple/control_dependency?^optimizer/gradients/optimizer/dense_4/BiasAdd_grad/BiasAddGrad
�
Koptimizer/gradients/optimizer/dense_4/BiasAdd_grad/tuple/control_dependencyIdentityMoptimizer/gradients/optimizer/SquaredDifference_grad/tuple/control_dependencyD^optimizer/gradients/optimizer/dense_4/BiasAdd_grad/tuple/group_deps*
T0*O
_classE
CAloc:@optimizer/gradients/optimizer/SquaredDifference_grad/Reshape
�
Moptimizer/gradients/optimizer/dense_4/BiasAdd_grad/tuple/control_dependency_1Identity>optimizer/gradients/optimizer/dense_4/BiasAdd_grad/BiasAddGradD^optimizer/gradients/optimizer/dense_4/BiasAdd_grad/tuple/group_deps*
T0*Q
_classG
ECloc:@optimizer/gradients/optimizer/dense_4/BiasAdd_grad/BiasAddGrad
�
1optimizer/gradients/optimizer/Log_grad/Reciprocal
Reciprocaloptimizer/add+^optimizer/gradients/optimizer/Neg_grad/Neg*
T0
�
*optimizer/gradients/optimizer/Log_grad/mulMul*optimizer/gradients/optimizer/Neg_grad/Neg1optimizer/gradients/optimizer/Log_grad/Reciprocal*
T0
�
8optimizer/gradients/optimizer/dense_4/MatMul_grad/MatMulMatMulKoptimizer/gradients/optimizer/dense_4/BiasAdd_grad/tuple/control_dependencyoptimizer//dense_4/kernel/read*
T0*
transpose_a( *
transpose_b(
�
:optimizer/gradients/optimizer/dense_4/MatMul_grad/MatMul_1MatMuloptimizer/dense_3/MulKoptimizer/gradients/optimizer/dense_4/BiasAdd_grad/tuple/control_dependency*
T0*
transpose_a(*
transpose_b( 
�
Boptimizer/gradients/optimizer/dense_4/MatMul_grad/tuple/group_depsNoOp9^optimizer/gradients/optimizer/dense_4/MatMul_grad/MatMul;^optimizer/gradients/optimizer/dense_4/MatMul_grad/MatMul_1
�
Joptimizer/gradients/optimizer/dense_4/MatMul_grad/tuple/control_dependencyIdentity8optimizer/gradients/optimizer/dense_4/MatMul_grad/MatMulC^optimizer/gradients/optimizer/dense_4/MatMul_grad/tuple/group_deps*
T0*K
_classA
?=loc:@optimizer/gradients/optimizer/dense_4/MatMul_grad/MatMul
�
Loptimizer/gradients/optimizer/dense_4/MatMul_grad/tuple/control_dependency_1Identity:optimizer/gradients/optimizer/dense_4/MatMul_grad/MatMul_1C^optimizer/gradients/optimizer/dense_4/MatMul_grad/tuple/group_deps*
T0*M
_classC
A?loc:@optimizer/gradients/optimizer/dense_4/MatMul_grad/MatMul_1
b
,optimizer/gradients/optimizer/add_grad/ShapeShapeoptimizer/concat_3*
T0*
out_type0
a
.optimizer/gradients/optimizer/add_grad/Shape_1Shapeoptimizer/add/y*
T0*
out_type0
�
<optimizer/gradients/optimizer/add_grad/BroadcastGradientArgsBroadcastGradientArgs,optimizer/gradients/optimizer/add_grad/Shape.optimizer/gradients/optimizer/add_grad/Shape_1*
T0
�
*optimizer/gradients/optimizer/add_grad/SumSum*optimizer/gradients/optimizer/Log_grad/mul<optimizer/gradients/optimizer/add_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
.optimizer/gradients/optimizer/add_grad/ReshapeReshape*optimizer/gradients/optimizer/add_grad/Sum,optimizer/gradients/optimizer/add_grad/Shape*
T0*
Tshape0
�
,optimizer/gradients/optimizer/add_grad/Sum_1Sum*optimizer/gradients/optimizer/Log_grad/mul>optimizer/gradients/optimizer/add_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
0optimizer/gradients/optimizer/add_grad/Reshape_1Reshape,optimizer/gradients/optimizer/add_grad/Sum_1.optimizer/gradients/optimizer/add_grad/Shape_1*
T0*
Tshape0
�
7optimizer/gradients/optimizer/add_grad/tuple/group_depsNoOp/^optimizer/gradients/optimizer/add_grad/Reshape1^optimizer/gradients/optimizer/add_grad/Reshape_1
�
?optimizer/gradients/optimizer/add_grad/tuple/control_dependencyIdentity.optimizer/gradients/optimizer/add_grad/Reshape8^optimizer/gradients/optimizer/add_grad/tuple/group_deps*
T0*A
_class7
53loc:@optimizer/gradients/optimizer/add_grad/Reshape
�
Aoptimizer/gradients/optimizer/add_grad/tuple/control_dependency_1Identity0optimizer/gradients/optimizer/add_grad/Reshape_18^optimizer/gradients/optimizer/add_grad/tuple/group_deps*
T0*C
_class9
75loc:@optimizer/gradients/optimizer/add_grad/Reshape_1
q
4optimizer/gradients/optimizer/dense_3/Mul_grad/ShapeShapeoptimizer/dense_3/BiasAdd*
T0*
out_type0
s
6optimizer/gradients/optimizer/dense_3/Mul_grad/Shape_1Shapeoptimizer/dense_3/Sigmoid*
T0*
out_type0
�
Doptimizer/gradients/optimizer/dense_3/Mul_grad/BroadcastGradientArgsBroadcastGradientArgs4optimizer/gradients/optimizer/dense_3/Mul_grad/Shape6optimizer/gradients/optimizer/dense_3/Mul_grad/Shape_1*
T0
�
2optimizer/gradients/optimizer/dense_3/Mul_grad/MulMulJoptimizer/gradients/optimizer/dense_4/MatMul_grad/tuple/control_dependencyoptimizer/dense_3/Sigmoid*
T0
�
2optimizer/gradients/optimizer/dense_3/Mul_grad/SumSum2optimizer/gradients/optimizer/dense_3/Mul_grad/MulDoptimizer/gradients/optimizer/dense_3/Mul_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
6optimizer/gradients/optimizer/dense_3/Mul_grad/ReshapeReshape2optimizer/gradients/optimizer/dense_3/Mul_grad/Sum4optimizer/gradients/optimizer/dense_3/Mul_grad/Shape*
T0*
Tshape0
�
4optimizer/gradients/optimizer/dense_3/Mul_grad/Mul_1Muloptimizer/dense_3/BiasAddJoptimizer/gradients/optimizer/dense_4/MatMul_grad/tuple/control_dependency*
T0
�
4optimizer/gradients/optimizer/dense_3/Mul_grad/Sum_1Sum4optimizer/gradients/optimizer/dense_3/Mul_grad/Mul_1Foptimizer/gradients/optimizer/dense_3/Mul_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
8optimizer/gradients/optimizer/dense_3/Mul_grad/Reshape_1Reshape4optimizer/gradients/optimizer/dense_3/Mul_grad/Sum_16optimizer/gradients/optimizer/dense_3/Mul_grad/Shape_1*
T0*
Tshape0
�
?optimizer/gradients/optimizer/dense_3/Mul_grad/tuple/group_depsNoOp7^optimizer/gradients/optimizer/dense_3/Mul_grad/Reshape9^optimizer/gradients/optimizer/dense_3/Mul_grad/Reshape_1
�
Goptimizer/gradients/optimizer/dense_3/Mul_grad/tuple/control_dependencyIdentity6optimizer/gradients/optimizer/dense_3/Mul_grad/Reshape@^optimizer/gradients/optimizer/dense_3/Mul_grad/tuple/group_deps*
T0*I
_class?
=;loc:@optimizer/gradients/optimizer/dense_3/Mul_grad/Reshape
�
Ioptimizer/gradients/optimizer/dense_3/Mul_grad/tuple/control_dependency_1Identity8optimizer/gradients/optimizer/dense_3/Mul_grad/Reshape_1@^optimizer/gradients/optimizer/dense_3/Mul_grad/tuple/group_deps*
T0*K
_classA
?=loc:@optimizer/gradients/optimizer/dense_3/Mul_grad/Reshape_1
Z
0optimizer/gradients/optimizer/concat_3_grad/RankConst*
dtype0*
value	B :
�
/optimizer/gradients/optimizer/concat_3_grad/modFloorModoptimizer/concat_3/axis0optimizer/gradients/optimizer/concat_3_grad/Rank*
T0
n
1optimizer/gradients/optimizer/concat_3_grad/ShapeShapeoptimizer/dense_1/Softmax*
T0*
out_type0
�
2optimizer/gradients/optimizer/concat_3_grad/ShapeNShapeNoptimizer/dense_1/Softmaxoptimizer/dense_2/Softmax*
N*
T0*
out_type0
�
8optimizer/gradients/optimizer/concat_3_grad/ConcatOffsetConcatOffset/optimizer/gradients/optimizer/concat_3_grad/mod2optimizer/gradients/optimizer/concat_3_grad/ShapeN4optimizer/gradients/optimizer/concat_3_grad/ShapeN:1*
N
�
1optimizer/gradients/optimizer/concat_3_grad/SliceSlice?optimizer/gradients/optimizer/add_grad/tuple/control_dependency8optimizer/gradients/optimizer/concat_3_grad/ConcatOffset2optimizer/gradients/optimizer/concat_3_grad/ShapeN*
Index0*
T0
�
3optimizer/gradients/optimizer/concat_3_grad/Slice_1Slice?optimizer/gradients/optimizer/add_grad/tuple/control_dependency:optimizer/gradients/optimizer/concat_3_grad/ConcatOffset:14optimizer/gradients/optimizer/concat_3_grad/ShapeN:1*
Index0*
T0
�
<optimizer/gradients/optimizer/concat_3_grad/tuple/group_depsNoOp2^optimizer/gradients/optimizer/concat_3_grad/Slice4^optimizer/gradients/optimizer/concat_3_grad/Slice_1
�
Doptimizer/gradients/optimizer/concat_3_grad/tuple/control_dependencyIdentity1optimizer/gradients/optimizer/concat_3_grad/Slice=^optimizer/gradients/optimizer/concat_3_grad/tuple/group_deps*
T0*D
_class:
86loc:@optimizer/gradients/optimizer/concat_3_grad/Slice
�
Foptimizer/gradients/optimizer/concat_3_grad/tuple/control_dependency_1Identity3optimizer/gradients/optimizer/concat_3_grad/Slice_1=^optimizer/gradients/optimizer/concat_3_grad/tuple/group_deps*
T0*F
_class<
:8loc:@optimizer/gradients/optimizer/concat_3_grad/Slice_1
�
>optimizer/gradients/optimizer/dense_3/Sigmoid_grad/SigmoidGradSigmoidGradoptimizer/dense_3/SigmoidIoptimizer/gradients/optimizer/dense_3/Mul_grad/tuple/control_dependency_1*
T0
�
6optimizer/gradients/optimizer/dense_1/Softmax_grad/mulMulDoptimizer/gradients/optimizer/concat_3_grad/tuple/control_dependencyoptimizer/dense_1/Softmax*
T0
{
Hoptimizer/gradients/optimizer/dense_1/Softmax_grad/Sum/reduction_indicesConst*
dtype0*
valueB :
���������
�
6optimizer/gradients/optimizer/dense_1/Softmax_grad/SumSum6optimizer/gradients/optimizer/dense_1/Softmax_grad/mulHoptimizer/gradients/optimizer/dense_1/Softmax_grad/Sum/reduction_indices*
T0*

Tidx0*
	keep_dims(
�
6optimizer/gradients/optimizer/dense_1/Softmax_grad/subSubDoptimizer/gradients/optimizer/concat_3_grad/tuple/control_dependency6optimizer/gradients/optimizer/dense_1/Softmax_grad/Sum*
T0
�
8optimizer/gradients/optimizer/dense_1/Softmax_grad/mul_1Mul6optimizer/gradients/optimizer/dense_1/Softmax_grad/suboptimizer/dense_1/Softmax*
T0
�
6optimizer/gradients/optimizer/dense_2/Softmax_grad/mulMulFoptimizer/gradients/optimizer/concat_3_grad/tuple/control_dependency_1optimizer/dense_2/Softmax*
T0
{
Hoptimizer/gradients/optimizer/dense_2/Softmax_grad/Sum/reduction_indicesConst*
dtype0*
valueB :
���������
�
6optimizer/gradients/optimizer/dense_2/Softmax_grad/SumSum6optimizer/gradients/optimizer/dense_2/Softmax_grad/mulHoptimizer/gradients/optimizer/dense_2/Softmax_grad/Sum/reduction_indices*
T0*

Tidx0*
	keep_dims(
�
6optimizer/gradients/optimizer/dense_2/Softmax_grad/subSubFoptimizer/gradients/optimizer/concat_3_grad/tuple/control_dependency_16optimizer/gradients/optimizer/dense_2/Softmax_grad/Sum*
T0
�
8optimizer/gradients/optimizer/dense_2/Softmax_grad/mul_1Mul6optimizer/gradients/optimizer/dense_2/Softmax_grad/suboptimizer/dense_2/Softmax*
T0
�
optimizer/gradients/AddNAddNGoptimizer/gradients/optimizer/dense_3/Mul_grad/tuple/control_dependency>optimizer/gradients/optimizer/dense_3/Sigmoid_grad/SigmoidGrad*
N*
T0*I
_class?
=;loc:@optimizer/gradients/optimizer/dense_3/Mul_grad/Reshape
�
>optimizer/gradients/optimizer/dense_3/BiasAdd_grad/BiasAddGradBiasAddGradoptimizer/gradients/AddN*
T0*
data_formatNHWC
�
Coptimizer/gradients/optimizer/dense_3/BiasAdd_grad/tuple/group_depsNoOp^optimizer/gradients/AddN?^optimizer/gradients/optimizer/dense_3/BiasAdd_grad/BiasAddGrad
�
Koptimizer/gradients/optimizer/dense_3/BiasAdd_grad/tuple/control_dependencyIdentityoptimizer/gradients/AddND^optimizer/gradients/optimizer/dense_3/BiasAdd_grad/tuple/group_deps*
T0*I
_class?
=;loc:@optimizer/gradients/optimizer/dense_3/Mul_grad/Reshape
�
Moptimizer/gradients/optimizer/dense_3/BiasAdd_grad/tuple/control_dependency_1Identity>optimizer/gradients/optimizer/dense_3/BiasAdd_grad/BiasAddGradD^optimizer/gradients/optimizer/dense_3/BiasAdd_grad/tuple/group_deps*
T0*Q
_classG
ECloc:@optimizer/gradients/optimizer/dense_3/BiasAdd_grad/BiasAddGrad
�
>optimizer/gradients/optimizer/dense_1/BiasAdd_grad/BiasAddGradBiasAddGrad8optimizer/gradients/optimizer/dense_1/Softmax_grad/mul_1*
T0*
data_formatNHWC
�
Coptimizer/gradients/optimizer/dense_1/BiasAdd_grad/tuple/group_depsNoOp?^optimizer/gradients/optimizer/dense_1/BiasAdd_grad/BiasAddGrad9^optimizer/gradients/optimizer/dense_1/Softmax_grad/mul_1
�
Koptimizer/gradients/optimizer/dense_1/BiasAdd_grad/tuple/control_dependencyIdentity8optimizer/gradients/optimizer/dense_1/Softmax_grad/mul_1D^optimizer/gradients/optimizer/dense_1/BiasAdd_grad/tuple/group_deps*
T0*K
_classA
?=loc:@optimizer/gradients/optimizer/dense_1/Softmax_grad/mul_1
�
Moptimizer/gradients/optimizer/dense_1/BiasAdd_grad/tuple/control_dependency_1Identity>optimizer/gradients/optimizer/dense_1/BiasAdd_grad/BiasAddGradD^optimizer/gradients/optimizer/dense_1/BiasAdd_grad/tuple/group_deps*
T0*Q
_classG
ECloc:@optimizer/gradients/optimizer/dense_1/BiasAdd_grad/BiasAddGrad
�
>optimizer/gradients/optimizer/dense_2/BiasAdd_grad/BiasAddGradBiasAddGrad8optimizer/gradients/optimizer/dense_2/Softmax_grad/mul_1*
T0*
data_formatNHWC
�
Coptimizer/gradients/optimizer/dense_2/BiasAdd_grad/tuple/group_depsNoOp?^optimizer/gradients/optimizer/dense_2/BiasAdd_grad/BiasAddGrad9^optimizer/gradients/optimizer/dense_2/Softmax_grad/mul_1
�
Koptimizer/gradients/optimizer/dense_2/BiasAdd_grad/tuple/control_dependencyIdentity8optimizer/gradients/optimizer/dense_2/Softmax_grad/mul_1D^optimizer/gradients/optimizer/dense_2/BiasAdd_grad/tuple/group_deps*
T0*K
_classA
?=loc:@optimizer/gradients/optimizer/dense_2/Softmax_grad/mul_1
�
Moptimizer/gradients/optimizer/dense_2/BiasAdd_grad/tuple/control_dependency_1Identity>optimizer/gradients/optimizer/dense_2/BiasAdd_grad/BiasAddGradD^optimizer/gradients/optimizer/dense_2/BiasAdd_grad/tuple/group_deps*
T0*Q
_classG
ECloc:@optimizer/gradients/optimizer/dense_2/BiasAdd_grad/BiasAddGrad
�
8optimizer/gradients/optimizer/dense_3/MatMul_grad/MatMulMatMulKoptimizer/gradients/optimizer/dense_3/BiasAdd_grad/tuple/control_dependencyoptimizer//dense_3/kernel/read*
T0*
transpose_a( *
transpose_b(
�
:optimizer/gradients/optimizer/dense_3/MatMul_grad/MatMul_1MatMuloptimizer/concat_4Koptimizer/gradients/optimizer/dense_3/BiasAdd_grad/tuple/control_dependency*
T0*
transpose_a(*
transpose_b( 
�
Boptimizer/gradients/optimizer/dense_3/MatMul_grad/tuple/group_depsNoOp9^optimizer/gradients/optimizer/dense_3/MatMul_grad/MatMul;^optimizer/gradients/optimizer/dense_3/MatMul_grad/MatMul_1
�
Joptimizer/gradients/optimizer/dense_3/MatMul_grad/tuple/control_dependencyIdentity8optimizer/gradients/optimizer/dense_3/MatMul_grad/MatMulC^optimizer/gradients/optimizer/dense_3/MatMul_grad/tuple/group_deps*
T0*K
_classA
?=loc:@optimizer/gradients/optimizer/dense_3/MatMul_grad/MatMul
�
Loptimizer/gradients/optimizer/dense_3/MatMul_grad/tuple/control_dependency_1Identity:optimizer/gradients/optimizer/dense_3/MatMul_grad/MatMul_1C^optimizer/gradients/optimizer/dense_3/MatMul_grad/tuple/group_deps*
T0*M
_classC
A?loc:@optimizer/gradients/optimizer/dense_3/MatMul_grad/MatMul_1
�
8optimizer/gradients/optimizer/dense_1/MatMul_grad/MatMulMatMulKoptimizer/gradients/optimizer/dense_1/BiasAdd_grad/tuple/control_dependencyoptimizer//dense_1/kernel/read*
T0*
transpose_a( *
transpose_b(
�
:optimizer/gradients/optimizer/dense_1/MatMul_grad/MatMul_1MatMuloptimizer/dense/MulKoptimizer/gradients/optimizer/dense_1/BiasAdd_grad/tuple/control_dependency*
T0*
transpose_a(*
transpose_b( 
�
Boptimizer/gradients/optimizer/dense_1/MatMul_grad/tuple/group_depsNoOp9^optimizer/gradients/optimizer/dense_1/MatMul_grad/MatMul;^optimizer/gradients/optimizer/dense_1/MatMul_grad/MatMul_1
�
Joptimizer/gradients/optimizer/dense_1/MatMul_grad/tuple/control_dependencyIdentity8optimizer/gradients/optimizer/dense_1/MatMul_grad/MatMulC^optimizer/gradients/optimizer/dense_1/MatMul_grad/tuple/group_deps*
T0*K
_classA
?=loc:@optimizer/gradients/optimizer/dense_1/MatMul_grad/MatMul
�
Loptimizer/gradients/optimizer/dense_1/MatMul_grad/tuple/control_dependency_1Identity:optimizer/gradients/optimizer/dense_1/MatMul_grad/MatMul_1C^optimizer/gradients/optimizer/dense_1/MatMul_grad/tuple/group_deps*
T0*M
_classC
A?loc:@optimizer/gradients/optimizer/dense_1/MatMul_grad/MatMul_1
�
8optimizer/gradients/optimizer/dense_2/MatMul_grad/MatMulMatMulKoptimizer/gradients/optimizer/dense_2/BiasAdd_grad/tuple/control_dependencyoptimizer//dense_2/kernel/read*
T0*
transpose_a( *
transpose_b(
�
:optimizer/gradients/optimizer/dense_2/MatMul_grad/MatMul_1MatMuloptimizer/dense/MulKoptimizer/gradients/optimizer/dense_2/BiasAdd_grad/tuple/control_dependency*
T0*
transpose_a(*
transpose_b( 
�
Boptimizer/gradients/optimizer/dense_2/MatMul_grad/tuple/group_depsNoOp9^optimizer/gradients/optimizer/dense_2/MatMul_grad/MatMul;^optimizer/gradients/optimizer/dense_2/MatMul_grad/MatMul_1
�
Joptimizer/gradients/optimizer/dense_2/MatMul_grad/tuple/control_dependencyIdentity8optimizer/gradients/optimizer/dense_2/MatMul_grad/MatMulC^optimizer/gradients/optimizer/dense_2/MatMul_grad/tuple/group_deps*
T0*K
_classA
?=loc:@optimizer/gradients/optimizer/dense_2/MatMul_grad/MatMul
�
Loptimizer/gradients/optimizer/dense_2/MatMul_grad/tuple/control_dependency_1Identity:optimizer/gradients/optimizer/dense_2/MatMul_grad/MatMul_1C^optimizer/gradients/optimizer/dense_2/MatMul_grad/tuple/group_deps*
T0*M
_classC
A?loc:@optimizer/gradients/optimizer/dense_2/MatMul_grad/MatMul_1
Z
0optimizer/gradients/optimizer/concat_4_grad/RankConst*
dtype0*
value	B :
�
/optimizer/gradients/optimizer/concat_4_grad/modFloorModoptimizer/concat_4/axis0optimizer/gradients/optimizer/concat_4_grad/Rank*
T0
l
1optimizer/gradients/optimizer/concat_4_grad/ShapeShapeoptimizer/concat/concat*
T0*
out_type0
�
2optimizer/gradients/optimizer/concat_4_grad/ShapeNShapeNoptimizer/concat/concatStopGradient*
N*
T0*
out_type0
�
8optimizer/gradients/optimizer/concat_4_grad/ConcatOffsetConcatOffset/optimizer/gradients/optimizer/concat_4_grad/mod2optimizer/gradients/optimizer/concat_4_grad/ShapeN4optimizer/gradients/optimizer/concat_4_grad/ShapeN:1*
N
�
1optimizer/gradients/optimizer/concat_4_grad/SliceSliceJoptimizer/gradients/optimizer/dense_3/MatMul_grad/tuple/control_dependency8optimizer/gradients/optimizer/concat_4_grad/ConcatOffset2optimizer/gradients/optimizer/concat_4_grad/ShapeN*
Index0*
T0
�
3optimizer/gradients/optimizer/concat_4_grad/Slice_1SliceJoptimizer/gradients/optimizer/dense_3/MatMul_grad/tuple/control_dependency:optimizer/gradients/optimizer/concat_4_grad/ConcatOffset:14optimizer/gradients/optimizer/concat_4_grad/ShapeN:1*
Index0*
T0
�
<optimizer/gradients/optimizer/concat_4_grad/tuple/group_depsNoOp2^optimizer/gradients/optimizer/concat_4_grad/Slice4^optimizer/gradients/optimizer/concat_4_grad/Slice_1
�
Doptimizer/gradients/optimizer/concat_4_grad/tuple/control_dependencyIdentity1optimizer/gradients/optimizer/concat_4_grad/Slice=^optimizer/gradients/optimizer/concat_4_grad/tuple/group_deps*
T0*D
_class:
86loc:@optimizer/gradients/optimizer/concat_4_grad/Slice
�
Foptimizer/gradients/optimizer/concat_4_grad/tuple/control_dependency_1Identity3optimizer/gradients/optimizer/concat_4_grad/Slice_1=^optimizer/gradients/optimizer/concat_4_grad/tuple/group_deps*
T0*F
_class<
:8loc:@optimizer/gradients/optimizer/concat_4_grad/Slice_1
�
optimizer/gradients/AddN_1AddNJoptimizer/gradients/optimizer/dense_1/MatMul_grad/tuple/control_dependencyJoptimizer/gradients/optimizer/dense_2/MatMul_grad/tuple/control_dependency*
N*
T0*K
_classA
?=loc:@optimizer/gradients/optimizer/dense_1/MatMul_grad/MatMul
m
2optimizer/gradients/optimizer/dense/Mul_grad/ShapeShapeoptimizer/dense/BiasAdd*
T0*
out_type0
o
4optimizer/gradients/optimizer/dense/Mul_grad/Shape_1Shapeoptimizer/dense/Sigmoid*
T0*
out_type0
�
Boptimizer/gradients/optimizer/dense/Mul_grad/BroadcastGradientArgsBroadcastGradientArgs2optimizer/gradients/optimizer/dense/Mul_grad/Shape4optimizer/gradients/optimizer/dense/Mul_grad/Shape_1*
T0
u
0optimizer/gradients/optimizer/dense/Mul_grad/MulMuloptimizer/gradients/AddN_1optimizer/dense/Sigmoid*
T0
�
0optimizer/gradients/optimizer/dense/Mul_grad/SumSum0optimizer/gradients/optimizer/dense/Mul_grad/MulBoptimizer/gradients/optimizer/dense/Mul_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
4optimizer/gradients/optimizer/dense/Mul_grad/ReshapeReshape0optimizer/gradients/optimizer/dense/Mul_grad/Sum2optimizer/gradients/optimizer/dense/Mul_grad/Shape*
T0*
Tshape0
w
2optimizer/gradients/optimizer/dense/Mul_grad/Mul_1Muloptimizer/dense/BiasAddoptimizer/gradients/AddN_1*
T0
�
2optimizer/gradients/optimizer/dense/Mul_grad/Sum_1Sum2optimizer/gradients/optimizer/dense/Mul_grad/Mul_1Doptimizer/gradients/optimizer/dense/Mul_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
6optimizer/gradients/optimizer/dense/Mul_grad/Reshape_1Reshape2optimizer/gradients/optimizer/dense/Mul_grad/Sum_14optimizer/gradients/optimizer/dense/Mul_grad/Shape_1*
T0*
Tshape0
�
=optimizer/gradients/optimizer/dense/Mul_grad/tuple/group_depsNoOp5^optimizer/gradients/optimizer/dense/Mul_grad/Reshape7^optimizer/gradients/optimizer/dense/Mul_grad/Reshape_1
�
Eoptimizer/gradients/optimizer/dense/Mul_grad/tuple/control_dependencyIdentity4optimizer/gradients/optimizer/dense/Mul_grad/Reshape>^optimizer/gradients/optimizer/dense/Mul_grad/tuple/group_deps*
T0*G
_class=
;9loc:@optimizer/gradients/optimizer/dense/Mul_grad/Reshape
�
Goptimizer/gradients/optimizer/dense/Mul_grad/tuple/control_dependency_1Identity6optimizer/gradients/optimizer/dense/Mul_grad/Reshape_1>^optimizer/gradients/optimizer/dense/Mul_grad/tuple/group_deps*
T0*I
_class?
=;loc:@optimizer/gradients/optimizer/dense/Mul_grad/Reshape_1
�
<optimizer/gradients/optimizer/dense/Sigmoid_grad/SigmoidGradSigmoidGradoptimizer/dense/SigmoidGoptimizer/gradients/optimizer/dense/Mul_grad/tuple/control_dependency_1*
T0
�
optimizer/gradients/AddN_2AddNEoptimizer/gradients/optimizer/dense/Mul_grad/tuple/control_dependency<optimizer/gradients/optimizer/dense/Sigmoid_grad/SigmoidGrad*
N*
T0*G
_class=
;9loc:@optimizer/gradients/optimizer/dense/Mul_grad/Reshape
�
<optimizer/gradients/optimizer/dense/BiasAdd_grad/BiasAddGradBiasAddGradoptimizer/gradients/AddN_2*
T0*
data_formatNHWC
�
Aoptimizer/gradients/optimizer/dense/BiasAdd_grad/tuple/group_depsNoOp^optimizer/gradients/AddN_2=^optimizer/gradients/optimizer/dense/BiasAdd_grad/BiasAddGrad
�
Ioptimizer/gradients/optimizer/dense/BiasAdd_grad/tuple/control_dependencyIdentityoptimizer/gradients/AddN_2B^optimizer/gradients/optimizer/dense/BiasAdd_grad/tuple/group_deps*
T0*G
_class=
;9loc:@optimizer/gradients/optimizer/dense/Mul_grad/Reshape
�
Koptimizer/gradients/optimizer/dense/BiasAdd_grad/tuple/control_dependency_1Identity<optimizer/gradients/optimizer/dense/BiasAdd_grad/BiasAddGradB^optimizer/gradients/optimizer/dense/BiasAdd_grad/tuple/group_deps*
T0*O
_classE
CAloc:@optimizer/gradients/optimizer/dense/BiasAdd_grad/BiasAddGrad
�
6optimizer/gradients/optimizer/dense/MatMul_grad/MatMulMatMulIoptimizer/gradients/optimizer/dense/BiasAdd_grad/tuple/control_dependencyoptimizer//dense/kernel/read*
T0*
transpose_a( *
transpose_b(
�
8optimizer/gradients/optimizer/dense/MatMul_grad/MatMul_1MatMuloptimizer/concat_2Ioptimizer/gradients/optimizer/dense/BiasAdd_grad/tuple/control_dependency*
T0*
transpose_a(*
transpose_b( 
�
@optimizer/gradients/optimizer/dense/MatMul_grad/tuple/group_depsNoOp7^optimizer/gradients/optimizer/dense/MatMul_grad/MatMul9^optimizer/gradients/optimizer/dense/MatMul_grad/MatMul_1
�
Hoptimizer/gradients/optimizer/dense/MatMul_grad/tuple/control_dependencyIdentity6optimizer/gradients/optimizer/dense/MatMul_grad/MatMulA^optimizer/gradients/optimizer/dense/MatMul_grad/tuple/group_deps*
T0*I
_class?
=;loc:@optimizer/gradients/optimizer/dense/MatMul_grad/MatMul
�
Joptimizer/gradients/optimizer/dense/MatMul_grad/tuple/control_dependency_1Identity8optimizer/gradients/optimizer/dense/MatMul_grad/MatMul_1A^optimizer/gradients/optimizer/dense/MatMul_grad/tuple/group_deps*
T0*K
_classA
?=loc:@optimizer/gradients/optimizer/dense/MatMul_grad/MatMul_1
Z
0optimizer/gradients/optimizer/concat_2_grad/RankConst*
dtype0*
value	B :
�
/optimizer/gradients/optimizer/concat_2_grad/modFloorModoptimizer/concat_2/axis0optimizer/gradients/optimizer/concat_2_grad/Rank*
T0
l
1optimizer/gradients/optimizer/concat_2_grad/ShapeShapeoptimizer/concat/concat*
T0*
out_type0
�
2optimizer/gradients/optimizer/concat_2_grad/ShapeNShapeNoptimizer/concat/concatoptimizer/concat_1/concat*
N*
T0*
out_type0
�
8optimizer/gradients/optimizer/concat_2_grad/ConcatOffsetConcatOffset/optimizer/gradients/optimizer/concat_2_grad/mod2optimizer/gradients/optimizer/concat_2_grad/ShapeN4optimizer/gradients/optimizer/concat_2_grad/ShapeN:1*
N
�
1optimizer/gradients/optimizer/concat_2_grad/SliceSliceHoptimizer/gradients/optimizer/dense/MatMul_grad/tuple/control_dependency8optimizer/gradients/optimizer/concat_2_grad/ConcatOffset2optimizer/gradients/optimizer/concat_2_grad/ShapeN*
Index0*
T0
�
3optimizer/gradients/optimizer/concat_2_grad/Slice_1SliceHoptimizer/gradients/optimizer/dense/MatMul_grad/tuple/control_dependency:optimizer/gradients/optimizer/concat_2_grad/ConcatOffset:14optimizer/gradients/optimizer/concat_2_grad/ShapeN:1*
Index0*
T0
�
<optimizer/gradients/optimizer/concat_2_grad/tuple/group_depsNoOp2^optimizer/gradients/optimizer/concat_2_grad/Slice4^optimizer/gradients/optimizer/concat_2_grad/Slice_1
�
Doptimizer/gradients/optimizer/concat_2_grad/tuple/control_dependencyIdentity1optimizer/gradients/optimizer/concat_2_grad/Slice=^optimizer/gradients/optimizer/concat_2_grad/tuple/group_deps*
T0*D
_class:
86loc:@optimizer/gradients/optimizer/concat_2_grad/Slice
�
Foptimizer/gradients/optimizer/concat_2_grad/tuple/control_dependency_1Identity3optimizer/gradients/optimizer/concat_2_grad/Slice_1=^optimizer/gradients/optimizer/concat_2_grad/tuple/group_deps*
T0*F
_class<
:8loc:@optimizer/gradients/optimizer/concat_2_grad/Slice_1
�
optimizer/gradients/AddN_3AddNDoptimizer/gradients/optimizer/concat_4_grad/tuple/control_dependencyDoptimizer/gradients/optimizer/concat_2_grad/tuple/control_dependency*
N*
T0*D
_class:
86loc:@optimizer/gradients/optimizer/concat_4_grad/Slice
�
optimizer/gradients/AddN_4AddNOoptimizer/gradients/optimizer/SquaredDifference_grad/tuple/control_dependency_1Foptimizer/gradients/optimizer/concat_2_grad/tuple/control_dependency_1*
N*
T0*K
_classA
?=loc:@optimizer/gradients/optimizer/SquaredDifference_grad/Neg
�
Roptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/Mul_grad/ShapeShape7optimizer/curiosity_vector_obs_encoder/hidden_1/BiasAdd*
T0*
out_type0
�
Toptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/Mul_grad/Shape_1Shape7optimizer/curiosity_vector_obs_encoder/hidden_1/Sigmoid*
T0*
out_type0
�
boptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/Mul_grad/BroadcastGradientArgsBroadcastGradientArgsRoptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/Mul_grad/ShapeToptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/Mul_grad/Shape_1*
T0
�
Poptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/Mul_grad/MulMuloptimizer/gradients/AddN_37optimizer/curiosity_vector_obs_encoder/hidden_1/Sigmoid*
T0
�
Poptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/Mul_grad/SumSumPoptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/Mul_grad/Mulboptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/Mul_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
Toptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/Mul_grad/ReshapeReshapePoptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/Mul_grad/SumRoptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/Mul_grad/Shape*
T0*
Tshape0
�
Roptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/Mul_grad/Mul_1Mul7optimizer/curiosity_vector_obs_encoder/hidden_1/BiasAddoptimizer/gradients/AddN_3*
T0
�
Roptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/Mul_grad/Sum_1SumRoptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/Mul_grad/Mul_1doptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/Mul_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
Voptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/Mul_grad/Reshape_1ReshapeRoptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/Mul_grad/Sum_1Toptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/Mul_grad/Shape_1*
T0*
Tshape0
�
]optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/Mul_grad/tuple/group_depsNoOpU^optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/Mul_grad/ReshapeW^optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/Mul_grad/Reshape_1
�
eoptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/Mul_grad/tuple/control_dependencyIdentityToptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/Mul_grad/Reshape^^optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/Mul_grad/tuple/group_deps*
T0*g
_class]
[Yloc:@optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/Mul_grad/Reshape
�
goptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/Mul_grad/tuple/control_dependency_1IdentityVoptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/Mul_grad/Reshape_1^^optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/Mul_grad/tuple/group_deps*
T0*i
_class_
][loc:@optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/Mul_grad/Reshape_1
�
Toptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/Mul_grad/ShapeShape9optimizer/curiosity_vector_obs_encoder_1/hidden_1/BiasAdd*
T0*
out_type0
�
Voptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/Mul_grad/Shape_1Shape9optimizer/curiosity_vector_obs_encoder_1/hidden_1/Sigmoid*
T0*
out_type0
�
doptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/Mul_grad/BroadcastGradientArgsBroadcastGradientArgsToptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/Mul_grad/ShapeVoptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/Mul_grad/Shape_1*
T0
�
Roptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/Mul_grad/MulMuloptimizer/gradients/AddN_49optimizer/curiosity_vector_obs_encoder_1/hidden_1/Sigmoid*
T0
�
Roptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/Mul_grad/SumSumRoptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/Mul_grad/Muldoptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/Mul_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
Voptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/Mul_grad/ReshapeReshapeRoptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/Mul_grad/SumToptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/Mul_grad/Shape*
T0*
Tshape0
�
Toptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/Mul_grad/Mul_1Mul9optimizer/curiosity_vector_obs_encoder_1/hidden_1/BiasAddoptimizer/gradients/AddN_4*
T0
�
Toptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/Mul_grad/Sum_1SumToptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/Mul_grad/Mul_1foptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/Mul_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
Xoptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/Mul_grad/Reshape_1ReshapeToptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/Mul_grad/Sum_1Voptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/Mul_grad/Shape_1*
T0*
Tshape0
�
_optimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/Mul_grad/tuple/group_depsNoOpW^optimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/Mul_grad/ReshapeY^optimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/Mul_grad/Reshape_1
�
goptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/Mul_grad/tuple/control_dependencyIdentityVoptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/Mul_grad/Reshape`^optimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/Mul_grad/tuple/group_deps*
T0*i
_class_
][loc:@optimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/Mul_grad/Reshape
�
ioptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/Mul_grad/tuple/control_dependency_1IdentityXoptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/Mul_grad/Reshape_1`^optimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/Mul_grad/tuple/group_deps*
T0*k
_classa
_]loc:@optimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/Mul_grad/Reshape_1
�
\optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/Sigmoid_grad/SigmoidGradSigmoidGrad7optimizer/curiosity_vector_obs_encoder/hidden_1/Sigmoidgoptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/Mul_grad/tuple/control_dependency_1*
T0
�
^optimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/Sigmoid_grad/SigmoidGradSigmoidGrad9optimizer/curiosity_vector_obs_encoder_1/hidden_1/Sigmoidioptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/Mul_grad/tuple/control_dependency_1*
T0
�
optimizer/gradients/AddN_5AddNeoptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/Mul_grad/tuple/control_dependency\optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/Sigmoid_grad/SigmoidGrad*
N*
T0*g
_class]
[Yloc:@optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/Mul_grad/Reshape
�
\optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/BiasAdd_grad/BiasAddGradBiasAddGradoptimizer/gradients/AddN_5*
T0*
data_formatNHWC
�
aoptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/BiasAdd_grad/tuple/group_depsNoOp^optimizer/gradients/AddN_5]^optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/BiasAdd_grad/BiasAddGrad
�
ioptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/BiasAdd_grad/tuple/control_dependencyIdentityoptimizer/gradients/AddN_5b^optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/BiasAdd_grad/tuple/group_deps*
T0*g
_class]
[Yloc:@optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/Mul_grad/Reshape
�
koptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/BiasAdd_grad/tuple/control_dependency_1Identity\optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/BiasAdd_grad/BiasAddGradb^optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/BiasAdd_grad/tuple/group_deps*
T0*o
_classe
caloc:@optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/BiasAdd_grad/BiasAddGrad
�
optimizer/gradients/AddN_6AddNgoptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/Mul_grad/tuple/control_dependency^optimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/Sigmoid_grad/SigmoidGrad*
N*
T0*i
_class_
][loc:@optimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/Mul_grad/Reshape
�
^optimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/BiasAdd_grad/BiasAddGradBiasAddGradoptimizer/gradients/AddN_6*
T0*
data_formatNHWC
�
coptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/BiasAdd_grad/tuple/group_depsNoOp^optimizer/gradients/AddN_6_^optimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/BiasAdd_grad/BiasAddGrad
�
koptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/BiasAdd_grad/tuple/control_dependencyIdentityoptimizer/gradients/AddN_6d^optimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/BiasAdd_grad/tuple/group_deps*
T0*i
_class_
][loc:@optimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/Mul_grad/Reshape
�
moptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/BiasAdd_grad/tuple/control_dependency_1Identity^optimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/BiasAdd_grad/BiasAddGradd^optimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/BiasAdd_grad/tuple/group_deps*
T0*q
_classg
ecloc:@optimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/BiasAdd_grad/BiasAddGrad
�
Voptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/MatMul_grad/MatMulMatMulioptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/BiasAdd_grad/tuple/control_dependency<optimizer//curiosity_vector_obs_encoder/hidden_1/kernel/read*
T0*
transpose_a( *
transpose_b(
�
Xoptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/MatMul_grad/MatMul_1MatMul3optimizer/curiosity_vector_obs_encoder/hidden_0/Mulioptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/BiasAdd_grad/tuple/control_dependency*
T0*
transpose_a(*
transpose_b( 
�
`optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/MatMul_grad/tuple/group_depsNoOpW^optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/MatMul_grad/MatMulY^optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/MatMul_grad/MatMul_1
�
hoptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/MatMul_grad/tuple/control_dependencyIdentityVoptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/MatMul_grad/MatMula^optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/MatMul_grad/tuple/group_deps*
T0*i
_class_
][loc:@optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/MatMul_grad/MatMul
�
joptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/MatMul_grad/tuple/control_dependency_1IdentityXoptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/MatMul_grad/MatMul_1a^optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/MatMul_grad/tuple/group_deps*
T0*k
_classa
_]loc:@optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/MatMul_grad/MatMul_1
�
Xoptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/MatMul_grad/MatMulMatMulkoptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/BiasAdd_grad/tuple/control_dependency<optimizer//curiosity_vector_obs_encoder/hidden_1/kernel/read*
T0*
transpose_a( *
transpose_b(
�
Zoptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/MatMul_grad/MatMul_1MatMul5optimizer/curiosity_vector_obs_encoder_1/hidden_0/Mulkoptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/BiasAdd_grad/tuple/control_dependency*
T0*
transpose_a(*
transpose_b( 
�
boptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/MatMul_grad/tuple/group_depsNoOpY^optimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/MatMul_grad/MatMul[^optimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/MatMul_grad/MatMul_1
�
joptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/MatMul_grad/tuple/control_dependencyIdentityXoptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/MatMul_grad/MatMulc^optimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/MatMul_grad/tuple/group_deps*
T0*k
_classa
_]loc:@optimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/MatMul_grad/MatMul
�
loptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/MatMul_grad/tuple/control_dependency_1IdentityZoptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/MatMul_grad/MatMul_1c^optimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/MatMul_grad/tuple/group_deps*
T0*m
_classc
a_loc:@optimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/MatMul_grad/MatMul_1
�
optimizer/gradients/AddN_7AddNkoptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/BiasAdd_grad/tuple/control_dependency_1moptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/BiasAdd_grad/tuple/control_dependency_1*
N*
T0*o
_classe
caloc:@optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/BiasAdd_grad/BiasAddGrad
�
Roptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/Mul_grad/ShapeShape7optimizer/curiosity_vector_obs_encoder/hidden_0/BiasAdd*
T0*
out_type0
�
Toptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/Mul_grad/Shape_1Shape7optimizer/curiosity_vector_obs_encoder/hidden_0/Sigmoid*
T0*
out_type0
�
boptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/Mul_grad/BroadcastGradientArgsBroadcastGradientArgsRoptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/Mul_grad/ShapeToptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/Mul_grad/Shape_1*
T0
�
Poptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/Mul_grad/MulMulhoptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/MatMul_grad/tuple/control_dependency7optimizer/curiosity_vector_obs_encoder/hidden_0/Sigmoid*
T0
�
Poptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/Mul_grad/SumSumPoptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/Mul_grad/Mulboptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/Mul_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
Toptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/Mul_grad/ReshapeReshapePoptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/Mul_grad/SumRoptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/Mul_grad/Shape*
T0*
Tshape0
�
Roptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/Mul_grad/Mul_1Mul7optimizer/curiosity_vector_obs_encoder/hidden_0/BiasAddhoptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/MatMul_grad/tuple/control_dependency*
T0
�
Roptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/Mul_grad/Sum_1SumRoptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/Mul_grad/Mul_1doptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/Mul_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
Voptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/Mul_grad/Reshape_1ReshapeRoptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/Mul_grad/Sum_1Toptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/Mul_grad/Shape_1*
T0*
Tshape0
�
]optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/Mul_grad/tuple/group_depsNoOpU^optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/Mul_grad/ReshapeW^optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/Mul_grad/Reshape_1
�
eoptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/Mul_grad/tuple/control_dependencyIdentityToptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/Mul_grad/Reshape^^optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/Mul_grad/tuple/group_deps*
T0*g
_class]
[Yloc:@optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/Mul_grad/Reshape
�
goptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/Mul_grad/tuple/control_dependency_1IdentityVoptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/Mul_grad/Reshape_1^^optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/Mul_grad/tuple/group_deps*
T0*i
_class_
][loc:@optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/Mul_grad/Reshape_1
�
Toptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/Mul_grad/ShapeShape9optimizer/curiosity_vector_obs_encoder_1/hidden_0/BiasAdd*
T0*
out_type0
�
Voptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/Mul_grad/Shape_1Shape9optimizer/curiosity_vector_obs_encoder_1/hidden_0/Sigmoid*
T0*
out_type0
�
doptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/Mul_grad/BroadcastGradientArgsBroadcastGradientArgsToptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/Mul_grad/ShapeVoptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/Mul_grad/Shape_1*
T0
�
Roptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/Mul_grad/MulMuljoptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/MatMul_grad/tuple/control_dependency9optimizer/curiosity_vector_obs_encoder_1/hidden_0/Sigmoid*
T0
�
Roptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/Mul_grad/SumSumRoptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/Mul_grad/Muldoptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/Mul_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
Voptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/Mul_grad/ReshapeReshapeRoptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/Mul_grad/SumToptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/Mul_grad/Shape*
T0*
Tshape0
�
Toptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/Mul_grad/Mul_1Mul9optimizer/curiosity_vector_obs_encoder_1/hidden_0/BiasAddjoptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/MatMul_grad/tuple/control_dependency*
T0
�
Toptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/Mul_grad/Sum_1SumToptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/Mul_grad/Mul_1foptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/Mul_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
Xoptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/Mul_grad/Reshape_1ReshapeToptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/Mul_grad/Sum_1Voptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/Mul_grad/Shape_1*
T0*
Tshape0
�
_optimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/Mul_grad/tuple/group_depsNoOpW^optimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/Mul_grad/ReshapeY^optimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/Mul_grad/Reshape_1
�
goptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/Mul_grad/tuple/control_dependencyIdentityVoptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/Mul_grad/Reshape`^optimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/Mul_grad/tuple/group_deps*
T0*i
_class_
][loc:@optimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/Mul_grad/Reshape
�
ioptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/Mul_grad/tuple/control_dependency_1IdentityXoptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/Mul_grad/Reshape_1`^optimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/Mul_grad/tuple/group_deps*
T0*k
_classa
_]loc:@optimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/Mul_grad/Reshape_1
�
optimizer/gradients/AddN_8AddNjoptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/MatMul_grad/tuple/control_dependency_1loptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_1/MatMul_grad/tuple/control_dependency_1*
N*
T0*k
_classa
_]loc:@optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_1/MatMul_grad/MatMul_1
�
\optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/Sigmoid_grad/SigmoidGradSigmoidGrad7optimizer/curiosity_vector_obs_encoder/hidden_0/Sigmoidgoptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/Mul_grad/tuple/control_dependency_1*
T0
�
^optimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/Sigmoid_grad/SigmoidGradSigmoidGrad9optimizer/curiosity_vector_obs_encoder_1/hidden_0/Sigmoidioptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/Mul_grad/tuple/control_dependency_1*
T0
�
optimizer/gradients/AddN_9AddNeoptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/Mul_grad/tuple/control_dependency\optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/Sigmoid_grad/SigmoidGrad*
N*
T0*g
_class]
[Yloc:@optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/Mul_grad/Reshape
�
\optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/BiasAdd_grad/BiasAddGradBiasAddGradoptimizer/gradients/AddN_9*
T0*
data_formatNHWC
�
aoptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/BiasAdd_grad/tuple/group_depsNoOp^optimizer/gradients/AddN_9]^optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/BiasAdd_grad/BiasAddGrad
�
ioptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/BiasAdd_grad/tuple/control_dependencyIdentityoptimizer/gradients/AddN_9b^optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/BiasAdd_grad/tuple/group_deps*
T0*g
_class]
[Yloc:@optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/Mul_grad/Reshape
�
koptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/BiasAdd_grad/tuple/control_dependency_1Identity\optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/BiasAdd_grad/BiasAddGradb^optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/BiasAdd_grad/tuple/group_deps*
T0*o
_classe
caloc:@optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/BiasAdd_grad/BiasAddGrad
�
optimizer/gradients/AddN_10AddNgoptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/Mul_grad/tuple/control_dependency^optimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/Sigmoid_grad/SigmoidGrad*
N*
T0*i
_class_
][loc:@optimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/Mul_grad/Reshape
�
^optimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/BiasAdd_grad/BiasAddGradBiasAddGradoptimizer/gradients/AddN_10*
T0*
data_formatNHWC
�
coptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/BiasAdd_grad/tuple/group_depsNoOp^optimizer/gradients/AddN_10_^optimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/BiasAdd_grad/BiasAddGrad
�
koptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/BiasAdd_grad/tuple/control_dependencyIdentityoptimizer/gradients/AddN_10d^optimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/BiasAdd_grad/tuple/group_deps*
T0*i
_class_
][loc:@optimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/Mul_grad/Reshape
�
moptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/BiasAdd_grad/tuple/control_dependency_1Identity^optimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/BiasAdd_grad/BiasAddGradd^optimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/BiasAdd_grad/tuple/group_deps*
T0*q
_classg
ecloc:@optimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/BiasAdd_grad/BiasAddGrad
�
Voptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/MatMul_grad/MatMulMatMulioptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/BiasAdd_grad/tuple/control_dependency<optimizer//curiosity_vector_obs_encoder/hidden_0/kernel/read*
T0*
transpose_a( *
transpose_b(
�
Xoptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/MatMul_grad/MatMul_1MatMulvector_observationioptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/BiasAdd_grad/tuple/control_dependency*
T0*
transpose_a(*
transpose_b( 
�
`optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/MatMul_grad/tuple/group_depsNoOpW^optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/MatMul_grad/MatMulY^optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/MatMul_grad/MatMul_1
�
hoptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/MatMul_grad/tuple/control_dependencyIdentityVoptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/MatMul_grad/MatMula^optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/MatMul_grad/tuple/group_deps*
T0*i
_class_
][loc:@optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/MatMul_grad/MatMul
�
joptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/MatMul_grad/tuple/control_dependency_1IdentityXoptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/MatMul_grad/MatMul_1a^optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/MatMul_grad/tuple/group_deps*
T0*k
_classa
_]loc:@optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/MatMul_grad/MatMul_1
�
Xoptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/MatMul_grad/MatMulMatMulkoptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/BiasAdd_grad/tuple/control_dependency<optimizer//curiosity_vector_obs_encoder/hidden_0/kernel/read*
T0*
transpose_a( *
transpose_b(
�
Zoptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/MatMul_grad/MatMul_1MatMul+optimizer/curiosity_next_vector_observationkoptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/BiasAdd_grad/tuple/control_dependency*
T0*
transpose_a(*
transpose_b( 
�
boptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/MatMul_grad/tuple/group_depsNoOpY^optimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/MatMul_grad/MatMul[^optimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/MatMul_grad/MatMul_1
�
joptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/MatMul_grad/tuple/control_dependencyIdentityXoptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/MatMul_grad/MatMulc^optimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/MatMul_grad/tuple/group_deps*
T0*k
_classa
_]loc:@optimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/MatMul_grad/MatMul
�
loptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/MatMul_grad/tuple/control_dependency_1IdentityZoptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/MatMul_grad/MatMul_1c^optimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/MatMul_grad/tuple/group_deps*
T0*m
_classc
a_loc:@optimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/MatMul_grad/MatMul_1
�
optimizer/gradients/AddN_11AddNkoptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/BiasAdd_grad/tuple/control_dependency_1moptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/BiasAdd_grad/tuple/control_dependency_1*
N*
T0*o
_classe
caloc:@optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/BiasAdd_grad/BiasAddGrad
�
optimizer/gradients/AddN_12AddNjoptimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/MatMul_grad/tuple/control_dependency_1loptimizer/gradients/optimizer/curiosity_vector_obs_encoder_1/hidden_0/MatMul_grad/tuple/control_dependency_1*
N*
T0*k
_classa
_]loc:@optimizer/gradients/optimizer/curiosity_vector_obs_encoder/hidden_0/MatMul_grad/MatMul_1
�
#optimizer/beta1_power/initial_valueConst*H
_class>
<:loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/bias*
dtype0*
valueB
 *fff?
�
optimizer/beta1_power
VariableV2*H
_class>
<:loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/bias*
	container *
dtype0*
shape: *
shared_name 
�
optimizer/beta1_power/AssignAssignoptimizer/beta1_power#optimizer/beta1_power/initial_value*
T0*H
_class>
<:loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/bias*
use_locking(*
validate_shape(
�
optimizer/beta1_power/readIdentityoptimizer/beta1_power*
T0*H
_class>
<:loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/bias
�
#optimizer/beta2_power/initial_valueConst*H
_class>
<:loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/bias*
dtype0*
valueB
 *w�?
�
optimizer/beta2_power
VariableV2*H
_class>
<:loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/bias*
	container *
dtype0*
shape: *
shared_name 
�
optimizer/beta2_power/AssignAssignoptimizer/beta2_power#optimizer/beta2_power/initial_value*
T0*H
_class>
<:loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/bias*
use_locking(*
validate_shape(
�
optimizer/beta2_power/readIdentityoptimizer/beta2_power*
T0*H
_class>
<:loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/bias
�
ioptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/kernel/Adam/Initializer/zeros/shape_as_tensorConst*J
_class@
><loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/kernel*
dtype0*
valueB"      
�
_optimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/kernel/Adam/Initializer/zeros/ConstConst*J
_class@
><loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/kernel*
dtype0*
valueB
 *    
�
Yoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/kernel/Adam/Initializer/zerosFillioptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/kernel/Adam/Initializer/zeros/shape_as_tensor_optimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/kernel/Adam/Initializer/zeros/Const*
T0*J
_class@
><loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/kernel*

index_type0
�
Goptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/kernel/Adam
VariableV2*J
_class@
><loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/kernel*
	container *
dtype0*
shape:	�*
shared_name 
�
Noptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/kernel/Adam/AssignAssignGoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/kernel/AdamYoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/kernel/Adam/Initializer/zeros*
T0*J
_class@
><loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/kernel*
use_locking(*
validate_shape(
�
Loptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/kernel/Adam/readIdentityGoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/kernel/Adam*
T0*J
_class@
><loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/kernel
�
koptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/kernel/Adam_1/Initializer/zeros/shape_as_tensorConst*J
_class@
><loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/kernel*
dtype0*
valueB"      
�
aoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/kernel/Adam_1/Initializer/zeros/ConstConst*J
_class@
><loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/kernel*
dtype0*
valueB
 *    
�
[optimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/kernel/Adam_1/Initializer/zerosFillkoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/kernel/Adam_1/Initializer/zeros/shape_as_tensoraoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/kernel/Adam_1/Initializer/zeros/Const*
T0*J
_class@
><loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/kernel*

index_type0
�
Ioptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/kernel/Adam_1
VariableV2*J
_class@
><loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/kernel*
	container *
dtype0*
shape:	�*
shared_name 
�
Poptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/kernel/Adam_1/AssignAssignIoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/kernel/Adam_1[optimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/kernel/Adam_1/Initializer/zeros*
T0*J
_class@
><loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/kernel*
use_locking(*
validate_shape(
�
Noptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/kernel/Adam_1/readIdentityIoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/kernel/Adam_1*
T0*J
_class@
><loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/kernel
�
Woptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/bias/Adam/Initializer/zerosConst*H
_class>
<:loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/bias*
dtype0*
valueB�*    
�
Eoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/bias/Adam
VariableV2*H
_class>
<:loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/bias*
	container *
dtype0*
shape:�*
shared_name 
�
Loptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/bias/Adam/AssignAssignEoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/bias/AdamWoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/bias/Adam/Initializer/zeros*
T0*H
_class>
<:loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/bias*
use_locking(*
validate_shape(
�
Joptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/bias/Adam/readIdentityEoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/bias/Adam*
T0*H
_class>
<:loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/bias
�
Yoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/bias/Adam_1/Initializer/zerosConst*H
_class>
<:loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/bias*
dtype0*
valueB�*    
�
Goptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/bias/Adam_1
VariableV2*H
_class>
<:loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/bias*
	container *
dtype0*
shape:�*
shared_name 
�
Noptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/bias/Adam_1/AssignAssignGoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/bias/Adam_1Yoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/bias/Adam_1/Initializer/zeros*
T0*H
_class>
<:loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/bias*
use_locking(*
validate_shape(
�
Loptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/bias/Adam_1/readIdentityGoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/bias/Adam_1*
T0*H
_class>
<:loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/bias
�
ioptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/kernel/Adam/Initializer/zeros/shape_as_tensorConst*J
_class@
><loc:@optimizer//curiosity_vector_obs_encoder/hidden_1/kernel*
dtype0*
valueB"      
�
_optimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/kernel/Adam/Initializer/zeros/ConstConst*J
_class@
><loc:@optimizer//curiosity_vector_obs_encoder/hidden_1/kernel*
dtype0*
valueB
 *    
�
Yoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/kernel/Adam/Initializer/zerosFillioptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/kernel/Adam/Initializer/zeros/shape_as_tensor_optimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/kernel/Adam/Initializer/zeros/Const*
T0*J
_class@
><loc:@optimizer//curiosity_vector_obs_encoder/hidden_1/kernel*

index_type0
�
Goptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/kernel/Adam
VariableV2*J
_class@
><loc:@optimizer//curiosity_vector_obs_encoder/hidden_1/kernel*
	container *
dtype0*
shape:
��*
shared_name 
�
Noptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/kernel/Adam/AssignAssignGoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/kernel/AdamYoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/kernel/Adam/Initializer/zeros*
T0*J
_class@
><loc:@optimizer//curiosity_vector_obs_encoder/hidden_1/kernel*
use_locking(*
validate_shape(
�
Loptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/kernel/Adam/readIdentityGoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/kernel/Adam*
T0*J
_class@
><loc:@optimizer//curiosity_vector_obs_encoder/hidden_1/kernel
�
koptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/kernel/Adam_1/Initializer/zeros/shape_as_tensorConst*J
_class@
><loc:@optimizer//curiosity_vector_obs_encoder/hidden_1/kernel*
dtype0*
valueB"      
�
aoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/kernel/Adam_1/Initializer/zeros/ConstConst*J
_class@
><loc:@optimizer//curiosity_vector_obs_encoder/hidden_1/kernel*
dtype0*
valueB
 *    
�
[optimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/kernel/Adam_1/Initializer/zerosFillkoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/kernel/Adam_1/Initializer/zeros/shape_as_tensoraoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/kernel/Adam_1/Initializer/zeros/Const*
T0*J
_class@
><loc:@optimizer//curiosity_vector_obs_encoder/hidden_1/kernel*

index_type0
�
Ioptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/kernel/Adam_1
VariableV2*J
_class@
><loc:@optimizer//curiosity_vector_obs_encoder/hidden_1/kernel*
	container *
dtype0*
shape:
��*
shared_name 
�
Poptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/kernel/Adam_1/AssignAssignIoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/kernel/Adam_1[optimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/kernel/Adam_1/Initializer/zeros*
T0*J
_class@
><loc:@optimizer//curiosity_vector_obs_encoder/hidden_1/kernel*
use_locking(*
validate_shape(
�
Noptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/kernel/Adam_1/readIdentityIoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/kernel/Adam_1*
T0*J
_class@
><loc:@optimizer//curiosity_vector_obs_encoder/hidden_1/kernel
�
Woptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/bias/Adam/Initializer/zerosConst*H
_class>
<:loc:@optimizer//curiosity_vector_obs_encoder/hidden_1/bias*
dtype0*
valueB�*    
�
Eoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/bias/Adam
VariableV2*H
_class>
<:loc:@optimizer//curiosity_vector_obs_encoder/hidden_1/bias*
	container *
dtype0*
shape:�*
shared_name 
�
Loptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/bias/Adam/AssignAssignEoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/bias/AdamWoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/bias/Adam/Initializer/zeros*
T0*H
_class>
<:loc:@optimizer//curiosity_vector_obs_encoder/hidden_1/bias*
use_locking(*
validate_shape(
�
Joptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/bias/Adam/readIdentityEoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/bias/Adam*
T0*H
_class>
<:loc:@optimizer//curiosity_vector_obs_encoder/hidden_1/bias
�
Yoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/bias/Adam_1/Initializer/zerosConst*H
_class>
<:loc:@optimizer//curiosity_vector_obs_encoder/hidden_1/bias*
dtype0*
valueB�*    
�
Goptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/bias/Adam_1
VariableV2*H
_class>
<:loc:@optimizer//curiosity_vector_obs_encoder/hidden_1/bias*
	container *
dtype0*
shape:�*
shared_name 
�
Noptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/bias/Adam_1/AssignAssignGoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/bias/Adam_1Yoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/bias/Adam_1/Initializer/zeros*
T0*H
_class>
<:loc:@optimizer//curiosity_vector_obs_encoder/hidden_1/bias*
use_locking(*
validate_shape(
�
Loptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/bias/Adam_1/readIdentityGoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/bias/Adam_1*
T0*H
_class>
<:loc:@optimizer//curiosity_vector_obs_encoder/hidden_1/bias
�
Ioptimizer//optimizer//dense/kernel/Adam/Initializer/zeros/shape_as_tensorConst**
_class 
loc:@optimizer//dense/kernel*
dtype0*
valueB"      
�
?optimizer//optimizer//dense/kernel/Adam/Initializer/zeros/ConstConst**
_class 
loc:@optimizer//dense/kernel*
dtype0*
valueB
 *    
�
9optimizer//optimizer//dense/kernel/Adam/Initializer/zerosFillIoptimizer//optimizer//dense/kernel/Adam/Initializer/zeros/shape_as_tensor?optimizer//optimizer//dense/kernel/Adam/Initializer/zeros/Const*
T0**
_class 
loc:@optimizer//dense/kernel*

index_type0
�
'optimizer//optimizer//dense/kernel/Adam
VariableV2**
_class 
loc:@optimizer//dense/kernel*
	container *
dtype0*
shape:
��*
shared_name 
�
.optimizer//optimizer//dense/kernel/Adam/AssignAssign'optimizer//optimizer//dense/kernel/Adam9optimizer//optimizer//dense/kernel/Adam/Initializer/zeros*
T0**
_class 
loc:@optimizer//dense/kernel*
use_locking(*
validate_shape(
�
,optimizer//optimizer//dense/kernel/Adam/readIdentity'optimizer//optimizer//dense/kernel/Adam*
T0**
_class 
loc:@optimizer//dense/kernel
�
Koptimizer//optimizer//dense/kernel/Adam_1/Initializer/zeros/shape_as_tensorConst**
_class 
loc:@optimizer//dense/kernel*
dtype0*
valueB"      
�
Aoptimizer//optimizer//dense/kernel/Adam_1/Initializer/zeros/ConstConst**
_class 
loc:@optimizer//dense/kernel*
dtype0*
valueB
 *    
�
;optimizer//optimizer//dense/kernel/Adam_1/Initializer/zerosFillKoptimizer//optimizer//dense/kernel/Adam_1/Initializer/zeros/shape_as_tensorAoptimizer//optimizer//dense/kernel/Adam_1/Initializer/zeros/Const*
T0**
_class 
loc:@optimizer//dense/kernel*

index_type0
�
)optimizer//optimizer//dense/kernel/Adam_1
VariableV2**
_class 
loc:@optimizer//dense/kernel*
	container *
dtype0*
shape:
��*
shared_name 
�
0optimizer//optimizer//dense/kernel/Adam_1/AssignAssign)optimizer//optimizer//dense/kernel/Adam_1;optimizer//optimizer//dense/kernel/Adam_1/Initializer/zeros*
T0**
_class 
loc:@optimizer//dense/kernel*
use_locking(*
validate_shape(
�
.optimizer//optimizer//dense/kernel/Adam_1/readIdentity)optimizer//optimizer//dense/kernel/Adam_1*
T0**
_class 
loc:@optimizer//dense/kernel
�
7optimizer//optimizer//dense/bias/Adam/Initializer/zerosConst*(
_class
loc:@optimizer//dense/bias*
dtype0*
valueB�*    
�
%optimizer//optimizer//dense/bias/Adam
VariableV2*(
_class
loc:@optimizer//dense/bias*
	container *
dtype0*
shape:�*
shared_name 
�
,optimizer//optimizer//dense/bias/Adam/AssignAssign%optimizer//optimizer//dense/bias/Adam7optimizer//optimizer//dense/bias/Adam/Initializer/zeros*
T0*(
_class
loc:@optimizer//dense/bias*
use_locking(*
validate_shape(
�
*optimizer//optimizer//dense/bias/Adam/readIdentity%optimizer//optimizer//dense/bias/Adam*
T0*(
_class
loc:@optimizer//dense/bias
�
9optimizer//optimizer//dense/bias/Adam_1/Initializer/zerosConst*(
_class
loc:@optimizer//dense/bias*
dtype0*
valueB�*    
�
'optimizer//optimizer//dense/bias/Adam_1
VariableV2*(
_class
loc:@optimizer//dense/bias*
	container *
dtype0*
shape:�*
shared_name 
�
.optimizer//optimizer//dense/bias/Adam_1/AssignAssign'optimizer//optimizer//dense/bias/Adam_19optimizer//optimizer//dense/bias/Adam_1/Initializer/zeros*
T0*(
_class
loc:@optimizer//dense/bias*
use_locking(*
validate_shape(
�
,optimizer//optimizer//dense/bias/Adam_1/readIdentity'optimizer//optimizer//dense/bias/Adam_1*
T0*(
_class
loc:@optimizer//dense/bias
�
;optimizer//optimizer//dense_1/kernel/Adam/Initializer/zerosConst*,
_class"
 loc:@optimizer//dense_1/kernel*
dtype0*
valueB	�*    
�
)optimizer//optimizer//dense_1/kernel/Adam
VariableV2*,
_class"
 loc:@optimizer//dense_1/kernel*
	container *
dtype0*
shape:	�*
shared_name 
�
0optimizer//optimizer//dense_1/kernel/Adam/AssignAssign)optimizer//optimizer//dense_1/kernel/Adam;optimizer//optimizer//dense_1/kernel/Adam/Initializer/zeros*
T0*,
_class"
 loc:@optimizer//dense_1/kernel*
use_locking(*
validate_shape(
�
.optimizer//optimizer//dense_1/kernel/Adam/readIdentity)optimizer//optimizer//dense_1/kernel/Adam*
T0*,
_class"
 loc:@optimizer//dense_1/kernel
�
=optimizer//optimizer//dense_1/kernel/Adam_1/Initializer/zerosConst*,
_class"
 loc:@optimizer//dense_1/kernel*
dtype0*
valueB	�*    
�
+optimizer//optimizer//dense_1/kernel/Adam_1
VariableV2*,
_class"
 loc:@optimizer//dense_1/kernel*
	container *
dtype0*
shape:	�*
shared_name 
�
2optimizer//optimizer//dense_1/kernel/Adam_1/AssignAssign+optimizer//optimizer//dense_1/kernel/Adam_1=optimizer//optimizer//dense_1/kernel/Adam_1/Initializer/zeros*
T0*,
_class"
 loc:@optimizer//dense_1/kernel*
use_locking(*
validate_shape(
�
0optimizer//optimizer//dense_1/kernel/Adam_1/readIdentity+optimizer//optimizer//dense_1/kernel/Adam_1*
T0*,
_class"
 loc:@optimizer//dense_1/kernel
�
9optimizer//optimizer//dense_1/bias/Adam/Initializer/zerosConst**
_class 
loc:@optimizer//dense_1/bias*
dtype0*
valueB*    
�
'optimizer//optimizer//dense_1/bias/Adam
VariableV2**
_class 
loc:@optimizer//dense_1/bias*
	container *
dtype0*
shape:*
shared_name 
�
.optimizer//optimizer//dense_1/bias/Adam/AssignAssign'optimizer//optimizer//dense_1/bias/Adam9optimizer//optimizer//dense_1/bias/Adam/Initializer/zeros*
T0**
_class 
loc:@optimizer//dense_1/bias*
use_locking(*
validate_shape(
�
,optimizer//optimizer//dense_1/bias/Adam/readIdentity'optimizer//optimizer//dense_1/bias/Adam*
T0**
_class 
loc:@optimizer//dense_1/bias
�
;optimizer//optimizer//dense_1/bias/Adam_1/Initializer/zerosConst**
_class 
loc:@optimizer//dense_1/bias*
dtype0*
valueB*    
�
)optimizer//optimizer//dense_1/bias/Adam_1
VariableV2**
_class 
loc:@optimizer//dense_1/bias*
	container *
dtype0*
shape:*
shared_name 
�
0optimizer//optimizer//dense_1/bias/Adam_1/AssignAssign)optimizer//optimizer//dense_1/bias/Adam_1;optimizer//optimizer//dense_1/bias/Adam_1/Initializer/zeros*
T0**
_class 
loc:@optimizer//dense_1/bias*
use_locking(*
validate_shape(
�
.optimizer//optimizer//dense_1/bias/Adam_1/readIdentity)optimizer//optimizer//dense_1/bias/Adam_1*
T0**
_class 
loc:@optimizer//dense_1/bias
�
;optimizer//optimizer//dense_2/kernel/Adam/Initializer/zerosConst*,
_class"
 loc:@optimizer//dense_2/kernel*
dtype0*
valueB	�*    
�
)optimizer//optimizer//dense_2/kernel/Adam
VariableV2*,
_class"
 loc:@optimizer//dense_2/kernel*
	container *
dtype0*
shape:	�*
shared_name 
�
0optimizer//optimizer//dense_2/kernel/Adam/AssignAssign)optimizer//optimizer//dense_2/kernel/Adam;optimizer//optimizer//dense_2/kernel/Adam/Initializer/zeros*
T0*,
_class"
 loc:@optimizer//dense_2/kernel*
use_locking(*
validate_shape(
�
.optimizer//optimizer//dense_2/kernel/Adam/readIdentity)optimizer//optimizer//dense_2/kernel/Adam*
T0*,
_class"
 loc:@optimizer//dense_2/kernel
�
=optimizer//optimizer//dense_2/kernel/Adam_1/Initializer/zerosConst*,
_class"
 loc:@optimizer//dense_2/kernel*
dtype0*
valueB	�*    
�
+optimizer//optimizer//dense_2/kernel/Adam_1
VariableV2*,
_class"
 loc:@optimizer//dense_2/kernel*
	container *
dtype0*
shape:	�*
shared_name 
�
2optimizer//optimizer//dense_2/kernel/Adam_1/AssignAssign+optimizer//optimizer//dense_2/kernel/Adam_1=optimizer//optimizer//dense_2/kernel/Adam_1/Initializer/zeros*
T0*,
_class"
 loc:@optimizer//dense_2/kernel*
use_locking(*
validate_shape(
�
0optimizer//optimizer//dense_2/kernel/Adam_1/readIdentity+optimizer//optimizer//dense_2/kernel/Adam_1*
T0*,
_class"
 loc:@optimizer//dense_2/kernel
�
9optimizer//optimizer//dense_2/bias/Adam/Initializer/zerosConst**
_class 
loc:@optimizer//dense_2/bias*
dtype0*
valueB*    
�
'optimizer//optimizer//dense_2/bias/Adam
VariableV2**
_class 
loc:@optimizer//dense_2/bias*
	container *
dtype0*
shape:*
shared_name 
�
.optimizer//optimizer//dense_2/bias/Adam/AssignAssign'optimizer//optimizer//dense_2/bias/Adam9optimizer//optimizer//dense_2/bias/Adam/Initializer/zeros*
T0**
_class 
loc:@optimizer//dense_2/bias*
use_locking(*
validate_shape(
�
,optimizer//optimizer//dense_2/bias/Adam/readIdentity'optimizer//optimizer//dense_2/bias/Adam*
T0**
_class 
loc:@optimizer//dense_2/bias
�
;optimizer//optimizer//dense_2/bias/Adam_1/Initializer/zerosConst**
_class 
loc:@optimizer//dense_2/bias*
dtype0*
valueB*    
�
)optimizer//optimizer//dense_2/bias/Adam_1
VariableV2**
_class 
loc:@optimizer//dense_2/bias*
	container *
dtype0*
shape:*
shared_name 
�
0optimizer//optimizer//dense_2/bias/Adam_1/AssignAssign)optimizer//optimizer//dense_2/bias/Adam_1;optimizer//optimizer//dense_2/bias/Adam_1/Initializer/zeros*
T0**
_class 
loc:@optimizer//dense_2/bias*
use_locking(*
validate_shape(
�
.optimizer//optimizer//dense_2/bias/Adam_1/readIdentity)optimizer//optimizer//dense_2/bias/Adam_1*
T0**
_class 
loc:@optimizer//dense_2/bias
�
Koptimizer//optimizer//dense_3/kernel/Adam/Initializer/zeros/shape_as_tensorConst*,
_class"
 loc:@optimizer//dense_3/kernel*
dtype0*
valueB"     
�
Aoptimizer//optimizer//dense_3/kernel/Adam/Initializer/zeros/ConstConst*,
_class"
 loc:@optimizer//dense_3/kernel*
dtype0*
valueB
 *    
�
;optimizer//optimizer//dense_3/kernel/Adam/Initializer/zerosFillKoptimizer//optimizer//dense_3/kernel/Adam/Initializer/zeros/shape_as_tensorAoptimizer//optimizer//dense_3/kernel/Adam/Initializer/zeros/Const*
T0*,
_class"
 loc:@optimizer//dense_3/kernel*

index_type0
�
)optimizer//optimizer//dense_3/kernel/Adam
VariableV2*,
_class"
 loc:@optimizer//dense_3/kernel*
	container *
dtype0*
shape:
��*
shared_name 
�
0optimizer//optimizer//dense_3/kernel/Adam/AssignAssign)optimizer//optimizer//dense_3/kernel/Adam;optimizer//optimizer//dense_3/kernel/Adam/Initializer/zeros*
T0*,
_class"
 loc:@optimizer//dense_3/kernel*
use_locking(*
validate_shape(
�
.optimizer//optimizer//dense_3/kernel/Adam/readIdentity)optimizer//optimizer//dense_3/kernel/Adam*
T0*,
_class"
 loc:@optimizer//dense_3/kernel
�
Moptimizer//optimizer//dense_3/kernel/Adam_1/Initializer/zeros/shape_as_tensorConst*,
_class"
 loc:@optimizer//dense_3/kernel*
dtype0*
valueB"     
�
Coptimizer//optimizer//dense_3/kernel/Adam_1/Initializer/zeros/ConstConst*,
_class"
 loc:@optimizer//dense_3/kernel*
dtype0*
valueB
 *    
�
=optimizer//optimizer//dense_3/kernel/Adam_1/Initializer/zerosFillMoptimizer//optimizer//dense_3/kernel/Adam_1/Initializer/zeros/shape_as_tensorCoptimizer//optimizer//dense_3/kernel/Adam_1/Initializer/zeros/Const*
T0*,
_class"
 loc:@optimizer//dense_3/kernel*

index_type0
�
+optimizer//optimizer//dense_3/kernel/Adam_1
VariableV2*,
_class"
 loc:@optimizer//dense_3/kernel*
	container *
dtype0*
shape:
��*
shared_name 
�
2optimizer//optimizer//dense_3/kernel/Adam_1/AssignAssign+optimizer//optimizer//dense_3/kernel/Adam_1=optimizer//optimizer//dense_3/kernel/Adam_1/Initializer/zeros*
T0*,
_class"
 loc:@optimizer//dense_3/kernel*
use_locking(*
validate_shape(
�
0optimizer//optimizer//dense_3/kernel/Adam_1/readIdentity+optimizer//optimizer//dense_3/kernel/Adam_1*
T0*,
_class"
 loc:@optimizer//dense_3/kernel
�
9optimizer//optimizer//dense_3/bias/Adam/Initializer/zerosConst**
_class 
loc:@optimizer//dense_3/bias*
dtype0*
valueB�*    
�
'optimizer//optimizer//dense_3/bias/Adam
VariableV2**
_class 
loc:@optimizer//dense_3/bias*
	container *
dtype0*
shape:�*
shared_name 
�
.optimizer//optimizer//dense_3/bias/Adam/AssignAssign'optimizer//optimizer//dense_3/bias/Adam9optimizer//optimizer//dense_3/bias/Adam/Initializer/zeros*
T0**
_class 
loc:@optimizer//dense_3/bias*
use_locking(*
validate_shape(
�
,optimizer//optimizer//dense_3/bias/Adam/readIdentity'optimizer//optimizer//dense_3/bias/Adam*
T0**
_class 
loc:@optimizer//dense_3/bias
�
;optimizer//optimizer//dense_3/bias/Adam_1/Initializer/zerosConst**
_class 
loc:@optimizer//dense_3/bias*
dtype0*
valueB�*    
�
)optimizer//optimizer//dense_3/bias/Adam_1
VariableV2**
_class 
loc:@optimizer//dense_3/bias*
	container *
dtype0*
shape:�*
shared_name 
�
0optimizer//optimizer//dense_3/bias/Adam_1/AssignAssign)optimizer//optimizer//dense_3/bias/Adam_1;optimizer//optimizer//dense_3/bias/Adam_1/Initializer/zeros*
T0**
_class 
loc:@optimizer//dense_3/bias*
use_locking(*
validate_shape(
�
.optimizer//optimizer//dense_3/bias/Adam_1/readIdentity)optimizer//optimizer//dense_3/bias/Adam_1*
T0**
_class 
loc:@optimizer//dense_3/bias
�
Koptimizer//optimizer//dense_4/kernel/Adam/Initializer/zeros/shape_as_tensorConst*,
_class"
 loc:@optimizer//dense_4/kernel*
dtype0*
valueB"      
�
Aoptimizer//optimizer//dense_4/kernel/Adam/Initializer/zeros/ConstConst*,
_class"
 loc:@optimizer//dense_4/kernel*
dtype0*
valueB
 *    
�
;optimizer//optimizer//dense_4/kernel/Adam/Initializer/zerosFillKoptimizer//optimizer//dense_4/kernel/Adam/Initializer/zeros/shape_as_tensorAoptimizer//optimizer//dense_4/kernel/Adam/Initializer/zeros/Const*
T0*,
_class"
 loc:@optimizer//dense_4/kernel*

index_type0
�
)optimizer//optimizer//dense_4/kernel/Adam
VariableV2*,
_class"
 loc:@optimizer//dense_4/kernel*
	container *
dtype0*
shape:
��*
shared_name 
�
0optimizer//optimizer//dense_4/kernel/Adam/AssignAssign)optimizer//optimizer//dense_4/kernel/Adam;optimizer//optimizer//dense_4/kernel/Adam/Initializer/zeros*
T0*,
_class"
 loc:@optimizer//dense_4/kernel*
use_locking(*
validate_shape(
�
.optimizer//optimizer//dense_4/kernel/Adam/readIdentity)optimizer//optimizer//dense_4/kernel/Adam*
T0*,
_class"
 loc:@optimizer//dense_4/kernel
�
Moptimizer//optimizer//dense_4/kernel/Adam_1/Initializer/zeros/shape_as_tensorConst*,
_class"
 loc:@optimizer//dense_4/kernel*
dtype0*
valueB"      
�
Coptimizer//optimizer//dense_4/kernel/Adam_1/Initializer/zeros/ConstConst*,
_class"
 loc:@optimizer//dense_4/kernel*
dtype0*
valueB
 *    
�
=optimizer//optimizer//dense_4/kernel/Adam_1/Initializer/zerosFillMoptimizer//optimizer//dense_4/kernel/Adam_1/Initializer/zeros/shape_as_tensorCoptimizer//optimizer//dense_4/kernel/Adam_1/Initializer/zeros/Const*
T0*,
_class"
 loc:@optimizer//dense_4/kernel*

index_type0
�
+optimizer//optimizer//dense_4/kernel/Adam_1
VariableV2*,
_class"
 loc:@optimizer//dense_4/kernel*
	container *
dtype0*
shape:
��*
shared_name 
�
2optimizer//optimizer//dense_4/kernel/Adam_1/AssignAssign+optimizer//optimizer//dense_4/kernel/Adam_1=optimizer//optimizer//dense_4/kernel/Adam_1/Initializer/zeros*
T0*,
_class"
 loc:@optimizer//dense_4/kernel*
use_locking(*
validate_shape(
�
0optimizer//optimizer//dense_4/kernel/Adam_1/readIdentity+optimizer//optimizer//dense_4/kernel/Adam_1*
T0*,
_class"
 loc:@optimizer//dense_4/kernel
�
9optimizer//optimizer//dense_4/bias/Adam/Initializer/zerosConst**
_class 
loc:@optimizer//dense_4/bias*
dtype0*
valueB�*    
�
'optimizer//optimizer//dense_4/bias/Adam
VariableV2**
_class 
loc:@optimizer//dense_4/bias*
	container *
dtype0*
shape:�*
shared_name 
�
.optimizer//optimizer//dense_4/bias/Adam/AssignAssign'optimizer//optimizer//dense_4/bias/Adam9optimizer//optimizer//dense_4/bias/Adam/Initializer/zeros*
T0**
_class 
loc:@optimizer//dense_4/bias*
use_locking(*
validate_shape(
�
,optimizer//optimizer//dense_4/bias/Adam/readIdentity'optimizer//optimizer//dense_4/bias/Adam*
T0**
_class 
loc:@optimizer//dense_4/bias
�
;optimizer//optimizer//dense_4/bias/Adam_1/Initializer/zerosConst**
_class 
loc:@optimizer//dense_4/bias*
dtype0*
valueB�*    
�
)optimizer//optimizer//dense_4/bias/Adam_1
VariableV2**
_class 
loc:@optimizer//dense_4/bias*
	container *
dtype0*
shape:�*
shared_name 
�
0optimizer//optimizer//dense_4/bias/Adam_1/AssignAssign)optimizer//optimizer//dense_4/bias/Adam_1;optimizer//optimizer//dense_4/bias/Adam_1/Initializer/zeros*
T0**
_class 
loc:@optimizer//dense_4/bias*
use_locking(*
validate_shape(
�
.optimizer//optimizer//dense_4/bias/Adam_1/readIdentity)optimizer//optimizer//dense_4/bias/Adam_1*
T0**
_class 
loc:@optimizer//dense_4/bias
I
optimizer/Adam/learning_rateConst*
dtype0*
valueB
 *o�:
A
optimizer/Adam/beta1Const*
dtype0*
valueB
 *fff?
A
optimizer/Adam/beta2Const*
dtype0*
valueB
 *w�?
C
optimizer/Adam/epsilonConst*
dtype0*
valueB
 *w�+2
�
Woptimizer/Adam/update_optimizer//curiosity_vector_obs_encoder/hidden_0/kernel/ApplyAdam	ApplyAdam7optimizer//curiosity_vector_obs_encoder/hidden_0/kernelGoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/kernel/AdamIoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/kernel/Adam_1optimizer/beta1_power/readoptimizer/beta2_power/readoptimizer/Adam/learning_rateoptimizer/Adam/beta1optimizer/Adam/beta2optimizer/Adam/epsilonoptimizer/gradients/AddN_12*
T0*J
_class@
><loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/kernel*
use_locking( *
use_nesterov( 
�
Uoptimizer/Adam/update_optimizer//curiosity_vector_obs_encoder/hidden_0/bias/ApplyAdam	ApplyAdam5optimizer//curiosity_vector_obs_encoder/hidden_0/biasEoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/bias/AdamGoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/bias/Adam_1optimizer/beta1_power/readoptimizer/beta2_power/readoptimizer/Adam/learning_rateoptimizer/Adam/beta1optimizer/Adam/beta2optimizer/Adam/epsilonoptimizer/gradients/AddN_11*
T0*H
_class>
<:loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/bias*
use_locking( *
use_nesterov( 
�
Woptimizer/Adam/update_optimizer//curiosity_vector_obs_encoder/hidden_1/kernel/ApplyAdam	ApplyAdam7optimizer//curiosity_vector_obs_encoder/hidden_1/kernelGoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/kernel/AdamIoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/kernel/Adam_1optimizer/beta1_power/readoptimizer/beta2_power/readoptimizer/Adam/learning_rateoptimizer/Adam/beta1optimizer/Adam/beta2optimizer/Adam/epsilonoptimizer/gradients/AddN_8*
T0*J
_class@
><loc:@optimizer//curiosity_vector_obs_encoder/hidden_1/kernel*
use_locking( *
use_nesterov( 
�
Uoptimizer/Adam/update_optimizer//curiosity_vector_obs_encoder/hidden_1/bias/ApplyAdam	ApplyAdam5optimizer//curiosity_vector_obs_encoder/hidden_1/biasEoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/bias/AdamGoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/bias/Adam_1optimizer/beta1_power/readoptimizer/beta2_power/readoptimizer/Adam/learning_rateoptimizer/Adam/beta1optimizer/Adam/beta2optimizer/Adam/epsilonoptimizer/gradients/AddN_7*
T0*H
_class>
<:loc:@optimizer//curiosity_vector_obs_encoder/hidden_1/bias*
use_locking( *
use_nesterov( 
�
7optimizer/Adam/update_optimizer//dense/kernel/ApplyAdam	ApplyAdamoptimizer//dense/kernel'optimizer//optimizer//dense/kernel/Adam)optimizer//optimizer//dense/kernel/Adam_1optimizer/beta1_power/readoptimizer/beta2_power/readoptimizer/Adam/learning_rateoptimizer/Adam/beta1optimizer/Adam/beta2optimizer/Adam/epsilonJoptimizer/gradients/optimizer/dense/MatMul_grad/tuple/control_dependency_1*
T0**
_class 
loc:@optimizer//dense/kernel*
use_locking( *
use_nesterov( 
�
5optimizer/Adam/update_optimizer//dense/bias/ApplyAdam	ApplyAdamoptimizer//dense/bias%optimizer//optimizer//dense/bias/Adam'optimizer//optimizer//dense/bias/Adam_1optimizer/beta1_power/readoptimizer/beta2_power/readoptimizer/Adam/learning_rateoptimizer/Adam/beta1optimizer/Adam/beta2optimizer/Adam/epsilonKoptimizer/gradients/optimizer/dense/BiasAdd_grad/tuple/control_dependency_1*
T0*(
_class
loc:@optimizer//dense/bias*
use_locking( *
use_nesterov( 
�
9optimizer/Adam/update_optimizer//dense_1/kernel/ApplyAdam	ApplyAdamoptimizer//dense_1/kernel)optimizer//optimizer//dense_1/kernel/Adam+optimizer//optimizer//dense_1/kernel/Adam_1optimizer/beta1_power/readoptimizer/beta2_power/readoptimizer/Adam/learning_rateoptimizer/Adam/beta1optimizer/Adam/beta2optimizer/Adam/epsilonLoptimizer/gradients/optimizer/dense_1/MatMul_grad/tuple/control_dependency_1*
T0*,
_class"
 loc:@optimizer//dense_1/kernel*
use_locking( *
use_nesterov( 
�
7optimizer/Adam/update_optimizer//dense_1/bias/ApplyAdam	ApplyAdamoptimizer//dense_1/bias'optimizer//optimizer//dense_1/bias/Adam)optimizer//optimizer//dense_1/bias/Adam_1optimizer/beta1_power/readoptimizer/beta2_power/readoptimizer/Adam/learning_rateoptimizer/Adam/beta1optimizer/Adam/beta2optimizer/Adam/epsilonMoptimizer/gradients/optimizer/dense_1/BiasAdd_grad/tuple/control_dependency_1*
T0**
_class 
loc:@optimizer//dense_1/bias*
use_locking( *
use_nesterov( 
�
9optimizer/Adam/update_optimizer//dense_2/kernel/ApplyAdam	ApplyAdamoptimizer//dense_2/kernel)optimizer//optimizer//dense_2/kernel/Adam+optimizer//optimizer//dense_2/kernel/Adam_1optimizer/beta1_power/readoptimizer/beta2_power/readoptimizer/Adam/learning_rateoptimizer/Adam/beta1optimizer/Adam/beta2optimizer/Adam/epsilonLoptimizer/gradients/optimizer/dense_2/MatMul_grad/tuple/control_dependency_1*
T0*,
_class"
 loc:@optimizer//dense_2/kernel*
use_locking( *
use_nesterov( 
�
7optimizer/Adam/update_optimizer//dense_2/bias/ApplyAdam	ApplyAdamoptimizer//dense_2/bias'optimizer//optimizer//dense_2/bias/Adam)optimizer//optimizer//dense_2/bias/Adam_1optimizer/beta1_power/readoptimizer/beta2_power/readoptimizer/Adam/learning_rateoptimizer/Adam/beta1optimizer/Adam/beta2optimizer/Adam/epsilonMoptimizer/gradients/optimizer/dense_2/BiasAdd_grad/tuple/control_dependency_1*
T0**
_class 
loc:@optimizer//dense_2/bias*
use_locking( *
use_nesterov( 
�
9optimizer/Adam/update_optimizer//dense_3/kernel/ApplyAdam	ApplyAdamoptimizer//dense_3/kernel)optimizer//optimizer//dense_3/kernel/Adam+optimizer//optimizer//dense_3/kernel/Adam_1optimizer/beta1_power/readoptimizer/beta2_power/readoptimizer/Adam/learning_rateoptimizer/Adam/beta1optimizer/Adam/beta2optimizer/Adam/epsilonLoptimizer/gradients/optimizer/dense_3/MatMul_grad/tuple/control_dependency_1*
T0*,
_class"
 loc:@optimizer//dense_3/kernel*
use_locking( *
use_nesterov( 
�
7optimizer/Adam/update_optimizer//dense_3/bias/ApplyAdam	ApplyAdamoptimizer//dense_3/bias'optimizer//optimizer//dense_3/bias/Adam)optimizer//optimizer//dense_3/bias/Adam_1optimizer/beta1_power/readoptimizer/beta2_power/readoptimizer/Adam/learning_rateoptimizer/Adam/beta1optimizer/Adam/beta2optimizer/Adam/epsilonMoptimizer/gradients/optimizer/dense_3/BiasAdd_grad/tuple/control_dependency_1*
T0**
_class 
loc:@optimizer//dense_3/bias*
use_locking( *
use_nesterov( 
�
9optimizer/Adam/update_optimizer//dense_4/kernel/ApplyAdam	ApplyAdamoptimizer//dense_4/kernel)optimizer//optimizer//dense_4/kernel/Adam+optimizer//optimizer//dense_4/kernel/Adam_1optimizer/beta1_power/readoptimizer/beta2_power/readoptimizer/Adam/learning_rateoptimizer/Adam/beta1optimizer/Adam/beta2optimizer/Adam/epsilonLoptimizer/gradients/optimizer/dense_4/MatMul_grad/tuple/control_dependency_1*
T0*,
_class"
 loc:@optimizer//dense_4/kernel*
use_locking( *
use_nesterov( 
�
7optimizer/Adam/update_optimizer//dense_4/bias/ApplyAdam	ApplyAdamoptimizer//dense_4/bias'optimizer//optimizer//dense_4/bias/Adam)optimizer//optimizer//dense_4/bias/Adam_1optimizer/beta1_power/readoptimizer/beta2_power/readoptimizer/Adam/learning_rateoptimizer/Adam/beta1optimizer/Adam/beta2optimizer/Adam/epsilonMoptimizer/gradients/optimizer/dense_4/BiasAdd_grad/tuple/control_dependency_1*
T0**
_class 
loc:@optimizer//dense_4/bias*
use_locking( *
use_nesterov( 
�
optimizer/Adam/mulMuloptimizer/beta1_power/readoptimizer/Adam/beta1V^optimizer/Adam/update_optimizer//curiosity_vector_obs_encoder/hidden_0/bias/ApplyAdamX^optimizer/Adam/update_optimizer//curiosity_vector_obs_encoder/hidden_0/kernel/ApplyAdamV^optimizer/Adam/update_optimizer//curiosity_vector_obs_encoder/hidden_1/bias/ApplyAdamX^optimizer/Adam/update_optimizer//curiosity_vector_obs_encoder/hidden_1/kernel/ApplyAdam6^optimizer/Adam/update_optimizer//dense/bias/ApplyAdam8^optimizer/Adam/update_optimizer//dense/kernel/ApplyAdam8^optimizer/Adam/update_optimizer//dense_1/bias/ApplyAdam:^optimizer/Adam/update_optimizer//dense_1/kernel/ApplyAdam8^optimizer/Adam/update_optimizer//dense_2/bias/ApplyAdam:^optimizer/Adam/update_optimizer//dense_2/kernel/ApplyAdam8^optimizer/Adam/update_optimizer//dense_3/bias/ApplyAdam:^optimizer/Adam/update_optimizer//dense_3/kernel/ApplyAdam8^optimizer/Adam/update_optimizer//dense_4/bias/ApplyAdam:^optimizer/Adam/update_optimizer//dense_4/kernel/ApplyAdam*
T0*H
_class>
<:loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/bias
�
optimizer/Adam/AssignAssignoptimizer/beta1_poweroptimizer/Adam/mul*
T0*H
_class>
<:loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/bias*
use_locking( *
validate_shape(
�
optimizer/Adam/mul_1Muloptimizer/beta2_power/readoptimizer/Adam/beta2V^optimizer/Adam/update_optimizer//curiosity_vector_obs_encoder/hidden_0/bias/ApplyAdamX^optimizer/Adam/update_optimizer//curiosity_vector_obs_encoder/hidden_0/kernel/ApplyAdamV^optimizer/Adam/update_optimizer//curiosity_vector_obs_encoder/hidden_1/bias/ApplyAdamX^optimizer/Adam/update_optimizer//curiosity_vector_obs_encoder/hidden_1/kernel/ApplyAdam6^optimizer/Adam/update_optimizer//dense/bias/ApplyAdam8^optimizer/Adam/update_optimizer//dense/kernel/ApplyAdam8^optimizer/Adam/update_optimizer//dense_1/bias/ApplyAdam:^optimizer/Adam/update_optimizer//dense_1/kernel/ApplyAdam8^optimizer/Adam/update_optimizer//dense_2/bias/ApplyAdam:^optimizer/Adam/update_optimizer//dense_2/kernel/ApplyAdam8^optimizer/Adam/update_optimizer//dense_3/bias/ApplyAdam:^optimizer/Adam/update_optimizer//dense_3/kernel/ApplyAdam8^optimizer/Adam/update_optimizer//dense_4/bias/ApplyAdam:^optimizer/Adam/update_optimizer//dense_4/kernel/ApplyAdam*
T0*H
_class>
<:loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/bias
�
optimizer/Adam/Assign_1Assignoptimizer/beta2_poweroptimizer/Adam/mul_1*
T0*H
_class>
<:loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/bias*
use_locking( *
validate_shape(
�
optimizer/AdamNoOp^optimizer/Adam/Assign^optimizer/Adam/Assign_1V^optimizer/Adam/update_optimizer//curiosity_vector_obs_encoder/hidden_0/bias/ApplyAdamX^optimizer/Adam/update_optimizer//curiosity_vector_obs_encoder/hidden_0/kernel/ApplyAdamV^optimizer/Adam/update_optimizer//curiosity_vector_obs_encoder/hidden_1/bias/ApplyAdamX^optimizer/Adam/update_optimizer//curiosity_vector_obs_encoder/hidden_1/kernel/ApplyAdam6^optimizer/Adam/update_optimizer//dense/bias/ApplyAdam8^optimizer/Adam/update_optimizer//dense/kernel/ApplyAdam8^optimizer/Adam/update_optimizer//dense_1/bias/ApplyAdam:^optimizer/Adam/update_optimizer//dense_1/kernel/ApplyAdam8^optimizer/Adam/update_optimizer//dense_2/bias/ApplyAdam:^optimizer/Adam/update_optimizer//dense_2/kernel/ApplyAdam8^optimizer/Adam/update_optimizer//dense_3/bias/ApplyAdam:^optimizer/Adam/update_optimizer//dense_3/kernel/ApplyAdam8^optimizer/Adam/update_optimizer//dense_4/bias/ApplyAdam:^optimizer/Adam/update_optimizer//dense_4/kernel/ApplyAdam
�
Joptimizer//main_graph_0/hidden_0/kernel/Initializer/truncated_normal/shapeConst*:
_class0
.,loc:@optimizer//main_graph_0/hidden_0/kernel*
dtype0*
valueB"   �   
�
Ioptimizer//main_graph_0/hidden_0/kernel/Initializer/truncated_normal/meanConst*:
_class0
.,loc:@optimizer//main_graph_0/hidden_0/kernel*
dtype0*
valueB
 *    
�
Koptimizer//main_graph_0/hidden_0/kernel/Initializer/truncated_normal/stddevConst*:
_class0
.,loc:@optimizer//main_graph_0/hidden_0/kernel*
dtype0*
valueB
 *���=
�
Toptimizer//main_graph_0/hidden_0/kernel/Initializer/truncated_normal/TruncatedNormalTruncatedNormalJoptimizer//main_graph_0/hidden_0/kernel/Initializer/truncated_normal/shape*
T0*:
_class0
.,loc:@optimizer//main_graph_0/hidden_0/kernel*
dtype0*
seed�0*
seed2
�
Hoptimizer//main_graph_0/hidden_0/kernel/Initializer/truncated_normal/mulMulToptimizer//main_graph_0/hidden_0/kernel/Initializer/truncated_normal/TruncatedNormalKoptimizer//main_graph_0/hidden_0/kernel/Initializer/truncated_normal/stddev*
T0*:
_class0
.,loc:@optimizer//main_graph_0/hidden_0/kernel
�
Doptimizer//main_graph_0/hidden_0/kernel/Initializer/truncated_normalAddHoptimizer//main_graph_0/hidden_0/kernel/Initializer/truncated_normal/mulIoptimizer//main_graph_0/hidden_0/kernel/Initializer/truncated_normal/mean*
T0*:
_class0
.,loc:@optimizer//main_graph_0/hidden_0/kernel
�
'optimizer//main_graph_0/hidden_0/kernel
VariableV2*:
_class0
.,loc:@optimizer//main_graph_0/hidden_0/kernel*
	container *
dtype0*
shape:	�*
shared_name 
�
.optimizer//main_graph_0/hidden_0/kernel/AssignAssign'optimizer//main_graph_0/hidden_0/kernelDoptimizer//main_graph_0/hidden_0/kernel/Initializer/truncated_normal*
T0*:
_class0
.,loc:@optimizer//main_graph_0/hidden_0/kernel*
use_locking(*
validate_shape(
�
,optimizer//main_graph_0/hidden_0/kernel/readIdentity'optimizer//main_graph_0/hidden_0/kernel*
T0*:
_class0
.,loc:@optimizer//main_graph_0/hidden_0/kernel
�
7optimizer//main_graph_0/hidden_0/bias/Initializer/zerosConst*8
_class.
,*loc:@optimizer//main_graph_0/hidden_0/bias*
dtype0*
valueB�*    
�
%optimizer//main_graph_0/hidden_0/bias
VariableV2*8
_class.
,*loc:@optimizer//main_graph_0/hidden_0/bias*
	container *
dtype0*
shape:�*
shared_name 
�
,optimizer//main_graph_0/hidden_0/bias/AssignAssign%optimizer//main_graph_0/hidden_0/bias7optimizer//main_graph_0/hidden_0/bias/Initializer/zeros*
T0*8
_class.
,*loc:@optimizer//main_graph_0/hidden_0/bias*
use_locking(*
validate_shape(
�
*optimizer//main_graph_0/hidden_0/bias/readIdentity%optimizer//main_graph_0/hidden_0/bias*
T0*8
_class.
,*loc:@optimizer//main_graph_0/hidden_0/bias
�
&optimizer/main_graph_0/hidden_0/MatMulMatMulvector_observation,optimizer//main_graph_0/hidden_0/kernel/read*
T0*
transpose_a( *
transpose_b( 
�
'optimizer/main_graph_0/hidden_0/BiasAddBiasAdd&optimizer/main_graph_0/hidden_0/MatMul*optimizer//main_graph_0/hidden_0/bias/read*
T0*
data_formatNHWC
d
'optimizer/main_graph_0/hidden_0/SigmoidSigmoid'optimizer/main_graph_0/hidden_0/BiasAdd*
T0
�
#optimizer/main_graph_0/hidden_0/MulMul'optimizer/main_graph_0/hidden_0/BiasAdd'optimizer/main_graph_0/hidden_0/Sigmoid*
T0
�
Joptimizer//main_graph_0/hidden_1/kernel/Initializer/truncated_normal/shapeConst*:
_class0
.,loc:@optimizer//main_graph_0/hidden_1/kernel*
dtype0*
valueB"�   �   
�
Ioptimizer//main_graph_0/hidden_1/kernel/Initializer/truncated_normal/meanConst*:
_class0
.,loc:@optimizer//main_graph_0/hidden_1/kernel*
dtype0*
valueB
 *    
�
Koptimizer//main_graph_0/hidden_1/kernel/Initializer/truncated_normal/stddevConst*:
_class0
.,loc:@optimizer//main_graph_0/hidden_1/kernel*
dtype0*
valueB
 *���=
�
Toptimizer//main_graph_0/hidden_1/kernel/Initializer/truncated_normal/TruncatedNormalTruncatedNormalJoptimizer//main_graph_0/hidden_1/kernel/Initializer/truncated_normal/shape*
T0*:
_class0
.,loc:@optimizer//main_graph_0/hidden_1/kernel*
dtype0*
seed�0*
seed2
�
Hoptimizer//main_graph_0/hidden_1/kernel/Initializer/truncated_normal/mulMulToptimizer//main_graph_0/hidden_1/kernel/Initializer/truncated_normal/TruncatedNormalKoptimizer//main_graph_0/hidden_1/kernel/Initializer/truncated_normal/stddev*
T0*:
_class0
.,loc:@optimizer//main_graph_0/hidden_1/kernel
�
Doptimizer//main_graph_0/hidden_1/kernel/Initializer/truncated_normalAddHoptimizer//main_graph_0/hidden_1/kernel/Initializer/truncated_normal/mulIoptimizer//main_graph_0/hidden_1/kernel/Initializer/truncated_normal/mean*
T0*:
_class0
.,loc:@optimizer//main_graph_0/hidden_1/kernel
�
'optimizer//main_graph_0/hidden_1/kernel
VariableV2*:
_class0
.,loc:@optimizer//main_graph_0/hidden_1/kernel*
	container *
dtype0*
shape:
��*
shared_name 
�
.optimizer//main_graph_0/hidden_1/kernel/AssignAssign'optimizer//main_graph_0/hidden_1/kernelDoptimizer//main_graph_0/hidden_1/kernel/Initializer/truncated_normal*
T0*:
_class0
.,loc:@optimizer//main_graph_0/hidden_1/kernel*
use_locking(*
validate_shape(
�
,optimizer//main_graph_0/hidden_1/kernel/readIdentity'optimizer//main_graph_0/hidden_1/kernel*
T0*:
_class0
.,loc:@optimizer//main_graph_0/hidden_1/kernel
�
7optimizer//main_graph_0/hidden_1/bias/Initializer/zerosConst*8
_class.
,*loc:@optimizer//main_graph_0/hidden_1/bias*
dtype0*
valueB�*    
�
%optimizer//main_graph_0/hidden_1/bias
VariableV2*8
_class.
,*loc:@optimizer//main_graph_0/hidden_1/bias*
	container *
dtype0*
shape:�*
shared_name 
�
,optimizer//main_graph_0/hidden_1/bias/AssignAssign%optimizer//main_graph_0/hidden_1/bias7optimizer//main_graph_0/hidden_1/bias/Initializer/zeros*
T0*8
_class.
,*loc:@optimizer//main_graph_0/hidden_1/bias*
use_locking(*
validate_shape(
�
*optimizer//main_graph_0/hidden_1/bias/readIdentity%optimizer//main_graph_0/hidden_1/bias*
T0*8
_class.
,*loc:@optimizer//main_graph_0/hidden_1/bias
�
&optimizer/main_graph_0/hidden_1/MatMulMatMul#optimizer/main_graph_0/hidden_0/Mul,optimizer//main_graph_0/hidden_1/kernel/read*
T0*
transpose_a( *
transpose_b( 
�
'optimizer/main_graph_0/hidden_1/BiasAddBiasAdd&optimizer/main_graph_0/hidden_1/MatMul*optimizer//main_graph_0/hidden_1/bias/read*
T0*
data_formatNHWC
d
'optimizer/main_graph_0/hidden_1/SigmoidSigmoid'optimizer/main_graph_0/hidden_1/BiasAdd*
T0
�
#optimizer/main_graph_0/hidden_1/MulMul'optimizer/main_graph_0/hidden_1/BiasAdd'optimizer/main_graph_0/hidden_1/Sigmoid*
T0
�
Boptimizer//extrinsic_value/kernel/Initializer/random_uniform/shapeConst*4
_class*
(&loc:@optimizer//extrinsic_value/kernel*
dtype0*
valueB"�      
�
@optimizer//extrinsic_value/kernel/Initializer/random_uniform/minConst*4
_class*
(&loc:@optimizer//extrinsic_value/kernel*
dtype0*
valueB
 *n�\�
�
@optimizer//extrinsic_value/kernel/Initializer/random_uniform/maxConst*4
_class*
(&loc:@optimizer//extrinsic_value/kernel*
dtype0*
valueB
 *n�\>
�
Joptimizer//extrinsic_value/kernel/Initializer/random_uniform/RandomUniformRandomUniformBoptimizer//extrinsic_value/kernel/Initializer/random_uniform/shape*
T0*4
_class*
(&loc:@optimizer//extrinsic_value/kernel*
dtype0*
seed�0*
seed2
�
@optimizer//extrinsic_value/kernel/Initializer/random_uniform/subSub@optimizer//extrinsic_value/kernel/Initializer/random_uniform/max@optimizer//extrinsic_value/kernel/Initializer/random_uniform/min*
T0*4
_class*
(&loc:@optimizer//extrinsic_value/kernel
�
@optimizer//extrinsic_value/kernel/Initializer/random_uniform/mulMulJoptimizer//extrinsic_value/kernel/Initializer/random_uniform/RandomUniform@optimizer//extrinsic_value/kernel/Initializer/random_uniform/sub*
T0*4
_class*
(&loc:@optimizer//extrinsic_value/kernel
�
<optimizer//extrinsic_value/kernel/Initializer/random_uniformAdd@optimizer//extrinsic_value/kernel/Initializer/random_uniform/mul@optimizer//extrinsic_value/kernel/Initializer/random_uniform/min*
T0*4
_class*
(&loc:@optimizer//extrinsic_value/kernel
�
!optimizer//extrinsic_value/kernel
VariableV2*4
_class*
(&loc:@optimizer//extrinsic_value/kernel*
	container *
dtype0*
shape:	�*
shared_name 
�
(optimizer//extrinsic_value/kernel/AssignAssign!optimizer//extrinsic_value/kernel<optimizer//extrinsic_value/kernel/Initializer/random_uniform*
T0*4
_class*
(&loc:@optimizer//extrinsic_value/kernel*
use_locking(*
validate_shape(
�
&optimizer//extrinsic_value/kernel/readIdentity!optimizer//extrinsic_value/kernel*
T0*4
_class*
(&loc:@optimizer//extrinsic_value/kernel
�
1optimizer//extrinsic_value/bias/Initializer/zerosConst*2
_class(
&$loc:@optimizer//extrinsic_value/bias*
dtype0*
valueB*    
�
optimizer//extrinsic_value/bias
VariableV2*2
_class(
&$loc:@optimizer//extrinsic_value/bias*
	container *
dtype0*
shape:*
shared_name 
�
&optimizer//extrinsic_value/bias/AssignAssignoptimizer//extrinsic_value/bias1optimizer//extrinsic_value/bias/Initializer/zeros*
T0*2
_class(
&$loc:@optimizer//extrinsic_value/bias*
use_locking(*
validate_shape(
�
$optimizer//extrinsic_value/bias/readIdentityoptimizer//extrinsic_value/bias*
T0*2
_class(
&$loc:@optimizer//extrinsic_value/bias
�
 optimizer/extrinsic_value/MatMulMatMul#optimizer/main_graph_0/hidden_1/Mul&optimizer//extrinsic_value/kernel/read*
T0*
transpose_a( *
transpose_b( 
�
!optimizer/extrinsic_value/BiasAddBiasAdd optimizer/extrinsic_value/MatMul$optimizer//extrinsic_value/bias/read*
T0*
data_formatNHWC
�
Boptimizer//curiosity_value/kernel/Initializer/random_uniform/shapeConst*4
_class*
(&loc:@optimizer//curiosity_value/kernel*
dtype0*
valueB"�      
�
@optimizer//curiosity_value/kernel/Initializer/random_uniform/minConst*4
_class*
(&loc:@optimizer//curiosity_value/kernel*
dtype0*
valueB
 *n�\�
�
@optimizer//curiosity_value/kernel/Initializer/random_uniform/maxConst*4
_class*
(&loc:@optimizer//curiosity_value/kernel*
dtype0*
valueB
 *n�\>
�
Joptimizer//curiosity_value/kernel/Initializer/random_uniform/RandomUniformRandomUniformBoptimizer//curiosity_value/kernel/Initializer/random_uniform/shape*
T0*4
_class*
(&loc:@optimizer//curiosity_value/kernel*
dtype0*
seed�0*
seed2
�
@optimizer//curiosity_value/kernel/Initializer/random_uniform/subSub@optimizer//curiosity_value/kernel/Initializer/random_uniform/max@optimizer//curiosity_value/kernel/Initializer/random_uniform/min*
T0*4
_class*
(&loc:@optimizer//curiosity_value/kernel
�
@optimizer//curiosity_value/kernel/Initializer/random_uniform/mulMulJoptimizer//curiosity_value/kernel/Initializer/random_uniform/RandomUniform@optimizer//curiosity_value/kernel/Initializer/random_uniform/sub*
T0*4
_class*
(&loc:@optimizer//curiosity_value/kernel
�
<optimizer//curiosity_value/kernel/Initializer/random_uniformAdd@optimizer//curiosity_value/kernel/Initializer/random_uniform/mul@optimizer//curiosity_value/kernel/Initializer/random_uniform/min*
T0*4
_class*
(&loc:@optimizer//curiosity_value/kernel
�
!optimizer//curiosity_value/kernel
VariableV2*4
_class*
(&loc:@optimizer//curiosity_value/kernel*
	container *
dtype0*
shape:	�*
shared_name 
�
(optimizer//curiosity_value/kernel/AssignAssign!optimizer//curiosity_value/kernel<optimizer//curiosity_value/kernel/Initializer/random_uniform*
T0*4
_class*
(&loc:@optimizer//curiosity_value/kernel*
use_locking(*
validate_shape(
�
&optimizer//curiosity_value/kernel/readIdentity!optimizer//curiosity_value/kernel*
T0*4
_class*
(&loc:@optimizer//curiosity_value/kernel
�
1optimizer//curiosity_value/bias/Initializer/zerosConst*2
_class(
&$loc:@optimizer//curiosity_value/bias*
dtype0*
valueB*    
�
optimizer//curiosity_value/bias
VariableV2*2
_class(
&$loc:@optimizer//curiosity_value/bias*
	container *
dtype0*
shape:*
shared_name 
�
&optimizer//curiosity_value/bias/AssignAssignoptimizer//curiosity_value/bias1optimizer//curiosity_value/bias/Initializer/zeros*
T0*2
_class(
&$loc:@optimizer//curiosity_value/bias*
use_locking(*
validate_shape(
�
$optimizer//curiosity_value/bias/readIdentityoptimizer//curiosity_value/bias*
T0*2
_class(
&$loc:@optimizer//curiosity_value/bias
�
 optimizer/curiosity_value/MatMulMatMul#optimizer/main_graph_0/hidden_1/Mul&optimizer//curiosity_value/kernel/read*
T0*
transpose_a( *
transpose_b( 
�
!optimizer/curiosity_value/BiasAddBiasAdd optimizer/curiosity_value/MatMul$optimizer//curiosity_value/bias/read*
T0*
data_formatNHWC
�
optimizer/Mean_2/inputPack!optimizer/extrinsic_value/BiasAdd!optimizer/curiosity_value/BiasAdd*
N*
T0*

axis 
L
"optimizer/Mean_2/reduction_indicesConst*
dtype0*
value	B : 
z
optimizer/Mean_2Meanoptimizer/Mean_2/input"optimizer/Mean_2/reduction_indices*
T0*

Tidx0*
	keep_dims( 
U
optimizer/old_probabilitiesPlaceholder*
dtype0*
shape:���������
R
optimizer/strided_slice/stackConst*
dtype0*
valueB"        
T
optimizer/strided_slice/stack_1Const*
dtype0*
valueB"       
T
optimizer/strided_slice/stack_2Const*
dtype0*
valueB"      
�
optimizer/strided_sliceStridedSliceoptimizer/old_probabilitiesoptimizer/strided_slice/stackoptimizer/strided_slice/stack_1optimizer/strided_slice/stack_2*
Index0*
T0*

begin_mask*
ellipsis_mask *
end_mask*
new_axis_mask *
shrink_axis_mask 
T
optimizer/strided_slice_1/stackConst*
dtype0*
valueB"       
V
!optimizer/strided_slice_1/stack_1Const*
dtype0*
valueB"       
V
!optimizer/strided_slice_1/stack_2Const*
dtype0*
valueB"      
�
optimizer/strided_slice_1StridedSliceoptimizer/old_probabilitiesoptimizer/strided_slice_1/stack!optimizer/strided_slice_1/stack_1!optimizer/strided_slice_1/stack_2*
Index0*
T0*

begin_mask*
ellipsis_mask *
end_mask*
new_axis_mask *
shrink_axis_mask 
T
optimizer/strided_slice_2/stackConst*
dtype0*
valueB"        
V
!optimizer/strided_slice_2/stack_1Const*
dtype0*
valueB"       
V
!optimizer/strided_slice_2/stack_2Const*
dtype0*
valueB"      
�
optimizer/strided_slice_2StridedSliceaction_masksoptimizer/strided_slice_2/stack!optimizer/strided_slice_2/stack_1!optimizer/strided_slice_2/stack_2*
Index0*
T0*

begin_mask*
ellipsis_mask *
end_mask*
new_axis_mask *
shrink_axis_mask 
T
optimizer/strided_slice_3/stackConst*
dtype0*
valueB"       
V
!optimizer/strided_slice_3/stack_1Const*
dtype0*
valueB"       
V
!optimizer/strided_slice_3/stack_2Const*
dtype0*
valueB"      
�
optimizer/strided_slice_3StridedSliceaction_masksoptimizer/strided_slice_3/stack!optimizer/strided_slice_3/stack_1!optimizer/strided_slice_3/stack_2*
Index0*
T0*

begin_mask*
ellipsis_mask *
end_mask*
new_axis_mask *
shrink_axis_mask 
>
optimizer/SoftmaxSoftmaxoptimizer/strided_slice*
T0
>
optimizer/add_2/yConst*
dtype0*
valueB
 *���3
G
optimizer/add_2AddV2optimizer/Softmaxoptimizer/add_2/y*
T0
K
optimizer/Mul_5Muloptimizer/add_2optimizer/strided_slice_2*
T0
B
optimizer/Softmax_1Softmaxoptimizer/strided_slice_1*
T0
>
optimizer/add_3/yConst*
dtype0*
valueB
 *���3
I
optimizer/add_3AddV2optimizer/Softmax_1optimizer/add_3/y*
T0
K
optimizer/Mul_6Muloptimizer/add_3optimizer/strided_slice_3*
T0
K
!optimizer/Sum_2/reduction_indicesConst*
dtype0*
value	B :
p
optimizer/Sum_2Sumoptimizer/Mul_5!optimizer/Sum_2/reduction_indices*
T0*

Tidx0*
	keep_dims(
G
optimizer/truedivRealDivoptimizer/Mul_5optimizer/Sum_2*
T0
K
!optimizer/Sum_3/reduction_indicesConst*
dtype0*
value	B :
p
optimizer/Sum_3Sumoptimizer/Mul_6!optimizer/Sum_3/reduction_indices*
T0*

Tidx0*
	keep_dims(
I
optimizer/truediv_1RealDivoptimizer/Mul_6optimizer/Sum_3*
T0
>
optimizer/add_4/yConst*
dtype0*
valueB
 *���3
G
optimizer/add_4AddV2optimizer/truedivoptimizer/add_4/y*
T0
0
optimizer/Log_1Logoptimizer/add_4*
T0
W
-optimizer/multinomial/Multinomial/num_samplesConst*
dtype0*
value	B :
�
!optimizer/multinomial/MultinomialMultinomialoptimizer/Log_1-optimizer/multinomial/Multinomial/num_samples*
T0*
output_dtype0	*
seed�0*
seed2
>
optimizer/add_5/yConst*
dtype0*
valueB
 *���3
I
optimizer/add_5AddV2optimizer/truediv_1optimizer/add_5/y*
T0
0
optimizer/Log_2Logoptimizer/add_5*
T0
Y
/optimizer/multinomial_1/Multinomial/num_samplesConst*
dtype0*
value	B :
�
#optimizer/multinomial_1/MultinomialMultinomialoptimizer/Log_2/optimizer/multinomial_1/Multinomial/num_samples*
T0*
output_dtype0	*
seed�0*
seed2
A
optimizer/concat_5/axisConst*
dtype0*
value	B :
�
optimizer/concat_5ConcatV2!optimizer/multinomial/Multinomial#optimizer/multinomial_1/Multinomialoptimizer/concat_5/axis*
N*
T0	*

Tidx0
A
optimizer/concat_6/axisConst*
dtype0*
value	B :
}
optimizer/concat_6ConcatV2optimizer/truedivoptimizer/truediv_1optimizer/concat_6/axis*
N*
T0*

Tidx0
>
optimizer/add_6/yConst*
dtype0*
valueB
 *���3
G
optimizer/add_6AddV2optimizer/truedivoptimizer/add_6/y*
T0
0
optimizer/Log_3Logoptimizer/add_6*
T0
>
optimizer/add_7/yConst*
dtype0*
valueB
 *���3
I
optimizer/add_7AddV2optimizer/truediv_1optimizer/add_7/y*
T0
0
optimizer/Log_4Logoptimizer/add_7*
T0
A
optimizer/concat_7/axisConst*
dtype0*
value	B :
w
optimizer/concat_7ConcatV2optimizer/Log_3optimizer/Log_4optimizer/concat_7/axis*
N*
T0*

Tidx0
T
optimizer/strided_slice_4/stackConst*
dtype0*
valueB"        
V
!optimizer/strided_slice_4/stack_1Const*
dtype0*
valueB"       
V
!optimizer/strided_slice_4/stack_2Const*
dtype0*
valueB"      
�
optimizer/strided_slice_4StridedSliceStopGradientoptimizer/strided_slice_4/stack!optimizer/strided_slice_4/stack_1!optimizer/strided_slice_4/stack_2*
Index0*
T0*

begin_mask*
ellipsis_mask *
end_mask*
new_axis_mask *
shrink_axis_mask 
T
optimizer/strided_slice_5/stackConst*
dtype0*
valueB"        
V
!optimizer/strided_slice_5/stack_1Const*
dtype0*
valueB"       
V
!optimizer/strided_slice_5/stack_2Const*
dtype0*
valueB"      
�
optimizer/strided_slice_5StridedSliceoptimizer/concat_7optimizer/strided_slice_5/stack!optimizer/strided_slice_5/stack_1!optimizer/strided_slice_5/stack_2*
Index0*
T0*

begin_mask*
ellipsis_mask *
end_mask*
new_axis_mask *
shrink_axis_mask 
Z
0optimizer/softmax_cross_entropy_with_logits/RankConst*
dtype0*
value	B :
n
1optimizer/softmax_cross_entropy_with_logits/ShapeShapeoptimizer/strided_slice_5*
T0*
out_type0
\
2optimizer/softmax_cross_entropy_with_logits/Rank_1Const*
dtype0*
value	B :
p
3optimizer/softmax_cross_entropy_with_logits/Shape_1Shapeoptimizer/strided_slice_5*
T0*
out_type0
[
1optimizer/softmax_cross_entropy_with_logits/Sub/yConst*
dtype0*
value	B :
�
/optimizer/softmax_cross_entropy_with_logits/SubSub2optimizer/softmax_cross_entropy_with_logits/Rank_11optimizer/softmax_cross_entropy_with_logits/Sub/y*
T0
�
7optimizer/softmax_cross_entropy_with_logits/Slice/beginPack/optimizer/softmax_cross_entropy_with_logits/Sub*
N*
T0*

axis 
d
6optimizer/softmax_cross_entropy_with_logits/Slice/sizeConst*
dtype0*
valueB:
�
1optimizer/softmax_cross_entropy_with_logits/SliceSlice3optimizer/softmax_cross_entropy_with_logits/Shape_17optimizer/softmax_cross_entropy_with_logits/Slice/begin6optimizer/softmax_cross_entropy_with_logits/Slice/size*
Index0*
T0
r
;optimizer/softmax_cross_entropy_with_logits/concat/values_0Const*
dtype0*
valueB:
���������
a
7optimizer/softmax_cross_entropy_with_logits/concat/axisConst*
dtype0*
value	B : 
�
2optimizer/softmax_cross_entropy_with_logits/concatConcatV2;optimizer/softmax_cross_entropy_with_logits/concat/values_01optimizer/softmax_cross_entropy_with_logits/Slice7optimizer/softmax_cross_entropy_with_logits/concat/axis*
N*
T0*

Tidx0
�
3optimizer/softmax_cross_entropy_with_logits/ReshapeReshapeoptimizer/strided_slice_52optimizer/softmax_cross_entropy_with_logits/concat*
T0*
Tshape0
\
2optimizer/softmax_cross_entropy_with_logits/Rank_2Const*
dtype0*
value	B :
p
3optimizer/softmax_cross_entropy_with_logits/Shape_2Shapeoptimizer/strided_slice_4*
T0*
out_type0
]
3optimizer/softmax_cross_entropy_with_logits/Sub_1/yConst*
dtype0*
value	B :
�
1optimizer/softmax_cross_entropy_with_logits/Sub_1Sub2optimizer/softmax_cross_entropy_with_logits/Rank_23optimizer/softmax_cross_entropy_with_logits/Sub_1/y*
T0
�
9optimizer/softmax_cross_entropy_with_logits/Slice_1/beginPack1optimizer/softmax_cross_entropy_with_logits/Sub_1*
N*
T0*

axis 
f
8optimizer/softmax_cross_entropy_with_logits/Slice_1/sizeConst*
dtype0*
valueB:
�
3optimizer/softmax_cross_entropy_with_logits/Slice_1Slice3optimizer/softmax_cross_entropy_with_logits/Shape_29optimizer/softmax_cross_entropy_with_logits/Slice_1/begin8optimizer/softmax_cross_entropy_with_logits/Slice_1/size*
Index0*
T0
t
=optimizer/softmax_cross_entropy_with_logits/concat_1/values_0Const*
dtype0*
valueB:
���������
c
9optimizer/softmax_cross_entropy_with_logits/concat_1/axisConst*
dtype0*
value	B : 
�
4optimizer/softmax_cross_entropy_with_logits/concat_1ConcatV2=optimizer/softmax_cross_entropy_with_logits/concat_1/values_03optimizer/softmax_cross_entropy_with_logits/Slice_19optimizer/softmax_cross_entropy_with_logits/concat_1/axis*
N*
T0*

Tidx0
�
5optimizer/softmax_cross_entropy_with_logits/Reshape_1Reshapeoptimizer/strided_slice_44optimizer/softmax_cross_entropy_with_logits/concat_1*
T0*
Tshape0
�
+optimizer/softmax_cross_entropy_with_logitsSoftmaxCrossEntropyWithLogits3optimizer/softmax_cross_entropy_with_logits/Reshape5optimizer/softmax_cross_entropy_with_logits/Reshape_1*
T0
]
3optimizer/softmax_cross_entropy_with_logits/Sub_2/yConst*
dtype0*
value	B :
�
1optimizer/softmax_cross_entropy_with_logits/Sub_2Sub0optimizer/softmax_cross_entropy_with_logits/Rank3optimizer/softmax_cross_entropy_with_logits/Sub_2/y*
T0
g
9optimizer/softmax_cross_entropy_with_logits/Slice_2/beginConst*
dtype0*
valueB: 
�
8optimizer/softmax_cross_entropy_with_logits/Slice_2/sizePack1optimizer/softmax_cross_entropy_with_logits/Sub_2*
N*
T0*

axis 
�
3optimizer/softmax_cross_entropy_with_logits/Slice_2Slice1optimizer/softmax_cross_entropy_with_logits/Shape9optimizer/softmax_cross_entropy_with_logits/Slice_2/begin8optimizer/softmax_cross_entropy_with_logits/Slice_2/size*
Index0*
T0
�
5optimizer/softmax_cross_entropy_with_logits/Reshape_2Reshape+optimizer/softmax_cross_entropy_with_logits3optimizer/softmax_cross_entropy_with_logits/Slice_2*
T0*
Tshape0
V
optimizer/Neg_1Neg5optimizer/softmax_cross_entropy_with_logits/Reshape_2*
T0
T
optimizer/strided_slice_6/stackConst*
dtype0*
valueB"       
V
!optimizer/strided_slice_6/stack_1Const*
dtype0*
valueB"       
V
!optimizer/strided_slice_6/stack_2Const*
dtype0*
valueB"      
�
optimizer/strided_slice_6StridedSliceStopGradientoptimizer/strided_slice_6/stack!optimizer/strided_slice_6/stack_1!optimizer/strided_slice_6/stack_2*
Index0*
T0*

begin_mask*
ellipsis_mask *
end_mask*
new_axis_mask *
shrink_axis_mask 
T
optimizer/strided_slice_7/stackConst*
dtype0*
valueB"       
V
!optimizer/strided_slice_7/stack_1Const*
dtype0*
valueB"       
V
!optimizer/strided_slice_7/stack_2Const*
dtype0*
valueB"      
�
optimizer/strided_slice_7StridedSliceoptimizer/concat_7optimizer/strided_slice_7/stack!optimizer/strided_slice_7/stack_1!optimizer/strided_slice_7/stack_2*
Index0*
T0*

begin_mask*
ellipsis_mask *
end_mask*
new_axis_mask *
shrink_axis_mask 
\
2optimizer/softmax_cross_entropy_with_logits_1/RankConst*
dtype0*
value	B :
p
3optimizer/softmax_cross_entropy_with_logits_1/ShapeShapeoptimizer/strided_slice_7*
T0*
out_type0
^
4optimizer/softmax_cross_entropy_with_logits_1/Rank_1Const*
dtype0*
value	B :
r
5optimizer/softmax_cross_entropy_with_logits_1/Shape_1Shapeoptimizer/strided_slice_7*
T0*
out_type0
]
3optimizer/softmax_cross_entropy_with_logits_1/Sub/yConst*
dtype0*
value	B :
�
1optimizer/softmax_cross_entropy_with_logits_1/SubSub4optimizer/softmax_cross_entropy_with_logits_1/Rank_13optimizer/softmax_cross_entropy_with_logits_1/Sub/y*
T0
�
9optimizer/softmax_cross_entropy_with_logits_1/Slice/beginPack1optimizer/softmax_cross_entropy_with_logits_1/Sub*
N*
T0*

axis 
f
8optimizer/softmax_cross_entropy_with_logits_1/Slice/sizeConst*
dtype0*
valueB:
�
3optimizer/softmax_cross_entropy_with_logits_1/SliceSlice5optimizer/softmax_cross_entropy_with_logits_1/Shape_19optimizer/softmax_cross_entropy_with_logits_1/Slice/begin8optimizer/softmax_cross_entropy_with_logits_1/Slice/size*
Index0*
T0
t
=optimizer/softmax_cross_entropy_with_logits_1/concat/values_0Const*
dtype0*
valueB:
���������
c
9optimizer/softmax_cross_entropy_with_logits_1/concat/axisConst*
dtype0*
value	B : 
�
4optimizer/softmax_cross_entropy_with_logits_1/concatConcatV2=optimizer/softmax_cross_entropy_with_logits_1/concat/values_03optimizer/softmax_cross_entropy_with_logits_1/Slice9optimizer/softmax_cross_entropy_with_logits_1/concat/axis*
N*
T0*

Tidx0
�
5optimizer/softmax_cross_entropy_with_logits_1/ReshapeReshapeoptimizer/strided_slice_74optimizer/softmax_cross_entropy_with_logits_1/concat*
T0*
Tshape0
^
4optimizer/softmax_cross_entropy_with_logits_1/Rank_2Const*
dtype0*
value	B :
r
5optimizer/softmax_cross_entropy_with_logits_1/Shape_2Shapeoptimizer/strided_slice_6*
T0*
out_type0
_
5optimizer/softmax_cross_entropy_with_logits_1/Sub_1/yConst*
dtype0*
value	B :
�
3optimizer/softmax_cross_entropy_with_logits_1/Sub_1Sub4optimizer/softmax_cross_entropy_with_logits_1/Rank_25optimizer/softmax_cross_entropy_with_logits_1/Sub_1/y*
T0
�
;optimizer/softmax_cross_entropy_with_logits_1/Slice_1/beginPack3optimizer/softmax_cross_entropy_with_logits_1/Sub_1*
N*
T0*

axis 
h
:optimizer/softmax_cross_entropy_with_logits_1/Slice_1/sizeConst*
dtype0*
valueB:
�
5optimizer/softmax_cross_entropy_with_logits_1/Slice_1Slice5optimizer/softmax_cross_entropy_with_logits_1/Shape_2;optimizer/softmax_cross_entropy_with_logits_1/Slice_1/begin:optimizer/softmax_cross_entropy_with_logits_1/Slice_1/size*
Index0*
T0
v
?optimizer/softmax_cross_entropy_with_logits_1/concat_1/values_0Const*
dtype0*
valueB:
���������
e
;optimizer/softmax_cross_entropy_with_logits_1/concat_1/axisConst*
dtype0*
value	B : 
�
6optimizer/softmax_cross_entropy_with_logits_1/concat_1ConcatV2?optimizer/softmax_cross_entropy_with_logits_1/concat_1/values_05optimizer/softmax_cross_entropy_with_logits_1/Slice_1;optimizer/softmax_cross_entropy_with_logits_1/concat_1/axis*
N*
T0*

Tidx0
�
7optimizer/softmax_cross_entropy_with_logits_1/Reshape_1Reshapeoptimizer/strided_slice_66optimizer/softmax_cross_entropy_with_logits_1/concat_1*
T0*
Tshape0
�
-optimizer/softmax_cross_entropy_with_logits_1SoftmaxCrossEntropyWithLogits5optimizer/softmax_cross_entropy_with_logits_1/Reshape7optimizer/softmax_cross_entropy_with_logits_1/Reshape_1*
T0
_
5optimizer/softmax_cross_entropy_with_logits_1/Sub_2/yConst*
dtype0*
value	B :
�
3optimizer/softmax_cross_entropy_with_logits_1/Sub_2Sub2optimizer/softmax_cross_entropy_with_logits_1/Rank5optimizer/softmax_cross_entropy_with_logits_1/Sub_2/y*
T0
i
;optimizer/softmax_cross_entropy_with_logits_1/Slice_2/beginConst*
dtype0*
valueB: 
�
:optimizer/softmax_cross_entropy_with_logits_1/Slice_2/sizePack3optimizer/softmax_cross_entropy_with_logits_1/Sub_2*
N*
T0*

axis 
�
5optimizer/softmax_cross_entropy_with_logits_1/Slice_2Slice3optimizer/softmax_cross_entropy_with_logits_1/Shape;optimizer/softmax_cross_entropy_with_logits_1/Slice_2/begin:optimizer/softmax_cross_entropy_with_logits_1/Slice_2/size*
Index0*
T0
�
7optimizer/softmax_cross_entropy_with_logits_1/Reshape_2Reshape-optimizer/softmax_cross_entropy_with_logits_15optimizer/softmax_cross_entropy_with_logits_1/Slice_2*
T0*
Tshape0
X
optimizer/Neg_2Neg7optimizer/softmax_cross_entropy_with_logits_1/Reshape_2*
T0
W
optimizer/stackPackoptimizer/Neg_1optimizer/Neg_2*
N*
T0*

axis
K
!optimizer/Sum_4/reduction_indicesConst*
dtype0*
value	B :
p
optimizer/Sum_4Sumoptimizer/stack!optimizer/Sum_4/reduction_indices*
T0*

Tidx0*
	keep_dims(
M
 optimizer/Variable/initial_valueConst*
dtype0*
valueB
 *RI�9
^
optimizer/Variable
VariableV2*
	container *
dtype0*
shape: *
shared_name 
�
optimizer/Variable/AssignAssignoptimizer/Variable optimizer/Variable/initial_value*
T0*%
_class
loc:@optimizer/Variable*
use_locking(*
validate_shape(
g
optimizer/Variable/readIdentityoptimizer/Variable*
T0*%
_class
loc:@optimizer/Variable
Q
optimizer/extrinsic_returnsPlaceholder*
dtype0*
shape:���������
X
"optimizer/extrinsic_value_estimatePlaceholder*
dtype0*
shape:���������
Q
optimizer/curiosity_returnsPlaceholder*
dtype0*
shape:���������
X
"optimizer/curiosity_value_estimatePlaceholder*
dtype0*
shape:���������
J
optimizer/advantagesPlaceholder*
dtype0*
shape:���������
K
optimizer/ExpandDims/dimConst*
dtype0*
valueB :
���������
g
optimizer/ExpandDims
ExpandDimsoptimizer/advantagesoptimizer/ExpandDims/dim*
T0*

Tdim0
O
"optimizer/Variable_1/initial_valueConst*
dtype0*
valueB
 *��L>
`
optimizer/Variable_1
VariableV2*
	container *
dtype0*
shape: *
shared_name 
�
optimizer/Variable_1/AssignAssignoptimizer/Variable_1"optimizer/Variable_1/initial_value*
T0*'
_class
loc:@optimizer/Variable_1*
use_locking(*
validate_shape(
m
optimizer/Variable_1/readIdentityoptimizer/Variable_1*
T0*'
_class
loc:@optimizer/Variable_1
O
"optimizer/Variable_2/initial_valueConst*
dtype0*
valueB
 *
ף;
`
optimizer/Variable_2
VariableV2*
	container *
dtype0*
shape: *
shared_name 
�
optimizer/Variable_2/AssignAssignoptimizer/Variable_2"optimizer/Variable_2/initial_value*
T0*'
_class
loc:@optimizer/Variable_2*
use_locking(*
validate_shape(
m
optimizer/Variable_2/readIdentityoptimizer/Variable_2*
T0*'
_class
loc:@optimizer/Variable_2
K
!optimizer/Sum_5/reduction_indicesConst*
dtype0*
value	B :
�
optimizer/Sum_5Sum!optimizer/extrinsic_value/BiasAdd!optimizer/Sum_5/reduction_indices*
T0*

Tidx0*
	keep_dims( 
R
optimizer/subSuboptimizer/Sum_5"optimizer/extrinsic_value_estimate*
T0
:
optimizer/Neg_3Negoptimizer/Variable_1/read*
T0
]
optimizer/clip_by_value/MinimumMinimumoptimizer/suboptimizer/Variable_1/read*
T0
]
optimizer/clip_by_valueMaximumoptimizer/clip_by_value/Minimumoptimizer/Neg_3*
T0
^
optimizer/add_8AddV2"optimizer/extrinsic_value_estimateoptimizer/clip_by_value*
T0
K
!optimizer/Sum_6/reduction_indicesConst*
dtype0*
value	B :
�
optimizer/Sum_6Sum!optimizer/extrinsic_value/BiasAdd!optimizer/Sum_6/reduction_indices*
T0*

Tidx0*
	keep_dims( 
i
optimizer/SquaredDifference_1SquaredDifferenceoptimizer/extrinsic_returnsoptimizer/Sum_6*
T0
i
optimizer/SquaredDifference_2SquaredDifferenceoptimizer/extrinsic_returnsoptimizer/add_8*
T0
c
optimizer/MaximumMaximumoptimizer/SquaredDifference_1optimizer/SquaredDifference_2*
T0
h
optimizer/DynamicPartition_2DynamicPartitionoptimizer/MaximumCast*
T0*
num_partitions
?
optimizer/Const_2Const*
dtype0*
valueB: 
q
optimizer/Mean_3Meanoptimizer/DynamicPartition_2:1optimizer/Const_2*
T0*

Tidx0*
	keep_dims( 
K
!optimizer/Sum_7/reduction_indicesConst*
dtype0*
value	B :
�
optimizer/Sum_7Sum!optimizer/curiosity_value/BiasAdd!optimizer/Sum_7/reduction_indices*
T0*

Tidx0*
	keep_dims( 
T
optimizer/sub_1Suboptimizer/Sum_7"optimizer/curiosity_value_estimate*
T0
:
optimizer/Neg_4Negoptimizer/Variable_1/read*
T0
a
!optimizer/clip_by_value_1/MinimumMinimumoptimizer/sub_1optimizer/Variable_1/read*
T0
a
optimizer/clip_by_value_1Maximum!optimizer/clip_by_value_1/Minimumoptimizer/Neg_4*
T0
`
optimizer/add_9AddV2"optimizer/curiosity_value_estimateoptimizer/clip_by_value_1*
T0
K
!optimizer/Sum_8/reduction_indicesConst*
dtype0*
value	B :
�
optimizer/Sum_8Sum!optimizer/curiosity_value/BiasAdd!optimizer/Sum_8/reduction_indices*
T0*

Tidx0*
	keep_dims( 
i
optimizer/SquaredDifference_3SquaredDifferenceoptimizer/curiosity_returnsoptimizer/Sum_8*
T0
i
optimizer/SquaredDifference_4SquaredDifferenceoptimizer/curiosity_returnsoptimizer/add_9*
T0
e
optimizer/Maximum_1Maximumoptimizer/SquaredDifference_3optimizer/SquaredDifference_4*
T0
j
optimizer/DynamicPartition_3DynamicPartitionoptimizer/Maximum_1Cast*
T0*
num_partitions
?
optimizer/Const_3Const*
dtype0*
valueB: 
q
optimizer/Mean_4Meanoptimizer/DynamicPartition_3:1optimizer/Const_3*
T0*

Tidx0*
	keep_dims( 
_
optimizer/Rank/packedPackoptimizer/Mean_3optimizer/Mean_4*
N*
T0*

axis 
8
optimizer/RankConst*
dtype0*
value	B :
?
optimizer/range/startConst*
dtype0*
value	B : 
?
optimizer/range/deltaConst*
dtype0*
value	B :
b
optimizer/rangeRangeoptimizer/range/startoptimizer/Rankoptimizer/range/delta*

Tidx0
`
optimizer/Mean_5/inputPackoptimizer/Mean_3optimizer/Mean_4*
N*
T0*

axis 
g
optimizer/Mean_5Meanoptimizer/Mean_5/inputoptimizer/range*
T0*

Tidx0*
	keep_dims( 
@
optimizer/sub_2Subpolicy_1/Sum_3optimizer/Sum_4*
T0
.
optimizer/ExpExpoptimizer/sub_2*
T0
D
optimizer/mul_7Muloptimizer/Expoptimizer/ExpandDims*
T0
>
optimizer/sub_3/xConst*
dtype0*
valueB
 *  �?
M
optimizer/sub_3Suboptimizer/sub_3/xoptimizer/Variable_1/read*
T0
?
optimizer/add_10/xConst*
dtype0*
valueB
 *  �?
Q
optimizer/add_10AddV2optimizer/add_10/xoptimizer/Variable_1/read*
T0
V
!optimizer/clip_by_value_2/MinimumMinimumoptimizer/Expoptimizer/add_10*
T0
a
optimizer/clip_by_value_2Maximum!optimizer/clip_by_value_2/Minimumoptimizer/sub_3*
T0
P
optimizer/mul_8Muloptimizer/clip_by_value_2optimizer/ExpandDims*
T0
G
optimizer/MinimumMinimumoptimizer/mul_7optimizer/mul_8*
T0
h
optimizer/DynamicPartition_4DynamicPartitionoptimizer/MinimumCast*
T0*
num_partitions
F
optimizer/Const_4Const*
dtype0*
valueB"       
q
optimizer/Mean_6Meanoptimizer/DynamicPartition_4:1optimizer/Const_4*
T0*

Tidx0*
	keep_dims( 
1
optimizer/Neg_5Negoptimizer/Mean_6*
T0
.
optimizer/AbsAbsoptimizer/Neg_5*
T0
>
optimizer/mul_9/xConst*
dtype0*
valueB
 *   ?
D
optimizer/mul_9Muloptimizer/mul_9/xoptimizer/Mean_5*
T0
D
optimizer/add_11AddV2optimizer/Neg_5optimizer/mul_9*
T0
e
optimizer/DynamicPartition_5DynamicPartitionpolicy_1/Sum_2Cast*
T0*
num_partitions
?
optimizer/Const_5Const*
dtype0*
valueB: 
q
optimizer/Mean_7Meanoptimizer/DynamicPartition_5:1optimizer/Const_5*
T0*

Tidx0*
	keep_dims( 
M
optimizer/mul_10Muloptimizer/Variable_2/readoptimizer/Mean_7*
T0
C
optimizer/sub_4Suboptimizer/add_11optimizer/mul_10*
T0
D
optimizer/gradients_1/ShapeConst*
dtype0*
valueB 
L
optimizer/gradients_1/grad_ys_0Const*
dtype0*
valueB
 *  �?
{
optimizer/gradients_1/FillFilloptimizer/gradients_1/Shapeoptimizer/gradients_1/grad_ys_0*
T0*

index_type0
Z
.optimizer/gradients_1/optimizer/sub_4_grad/NegNegoptimizer/gradients_1/Fill*
T0
�
;optimizer/gradients_1/optimizer/sub_4_grad/tuple/group_depsNoOp^optimizer/gradients_1/Fill/^optimizer/gradients_1/optimizer/sub_4_grad/Neg
�
Coptimizer/gradients_1/optimizer/sub_4_grad/tuple/control_dependencyIdentityoptimizer/gradients_1/Fill<^optimizer/gradients_1/optimizer/sub_4_grad/tuple/group_deps*
T0*-
_class#
!loc:@optimizer/gradients_1/Fill
�
Eoptimizer/gradients_1/optimizer/sub_4_grad/tuple/control_dependency_1Identity.optimizer/gradients_1/optimizer/sub_4_grad/Neg<^optimizer/gradients_1/optimizer/sub_4_grad/tuple/group_deps*
T0*A
_class7
53loc:@optimizer/gradients_1/optimizer/sub_4_grad/Neg
�
<optimizer/gradients_1/optimizer/add_11_grad/tuple/group_depsNoOpD^optimizer/gradients_1/optimizer/sub_4_grad/tuple/control_dependency
�
Doptimizer/gradients_1/optimizer/add_11_grad/tuple/control_dependencyIdentityCoptimizer/gradients_1/optimizer/sub_4_grad/tuple/control_dependency=^optimizer/gradients_1/optimizer/add_11_grad/tuple/group_deps*
T0*-
_class#
!loc:@optimizer/gradients_1/Fill
�
Foptimizer/gradients_1/optimizer/add_11_grad/tuple/control_dependency_1IdentityCoptimizer/gradients_1/optimizer/sub_4_grad/tuple/control_dependency=^optimizer/gradients_1/optimizer/add_11_grad/tuple/group_deps*
T0*-
_class#
!loc:@optimizer/gradients_1/Fill
�
/optimizer/gradients_1/optimizer/mul_10_grad/MulMulEoptimizer/gradients_1/optimizer/sub_4_grad/tuple/control_dependency_1optimizer/Mean_7*
T0
�
1optimizer/gradients_1/optimizer/mul_10_grad/Mul_1MulEoptimizer/gradients_1/optimizer/sub_4_grad/tuple/control_dependency_1optimizer/Variable_2/read*
T0
�
<optimizer/gradients_1/optimizer/mul_10_grad/tuple/group_depsNoOp0^optimizer/gradients_1/optimizer/mul_10_grad/Mul2^optimizer/gradients_1/optimizer/mul_10_grad/Mul_1
�
Doptimizer/gradients_1/optimizer/mul_10_grad/tuple/control_dependencyIdentity/optimizer/gradients_1/optimizer/mul_10_grad/Mul=^optimizer/gradients_1/optimizer/mul_10_grad/tuple/group_deps*
T0*B
_class8
64loc:@optimizer/gradients_1/optimizer/mul_10_grad/Mul
�
Foptimizer/gradients_1/optimizer/mul_10_grad/tuple/control_dependency_1Identity1optimizer/gradients_1/optimizer/mul_10_grad/Mul_1=^optimizer/gradients_1/optimizer/mul_10_grad/tuple/group_deps*
T0*D
_class:
86loc:@optimizer/gradients_1/optimizer/mul_10_grad/Mul_1
�
.optimizer/gradients_1/optimizer/Neg_5_grad/NegNegDoptimizer/gradients_1/optimizer/add_11_grad/tuple/control_dependency*
T0
�
.optimizer/gradients_1/optimizer/mul_9_grad/MulMulFoptimizer/gradients_1/optimizer/add_11_grad/tuple/control_dependency_1optimizer/Mean_5*
T0
�
0optimizer/gradients_1/optimizer/mul_9_grad/Mul_1MulFoptimizer/gradients_1/optimizer/add_11_grad/tuple/control_dependency_1optimizer/mul_9/x*
T0
�
;optimizer/gradients_1/optimizer/mul_9_grad/tuple/group_depsNoOp/^optimizer/gradients_1/optimizer/mul_9_grad/Mul1^optimizer/gradients_1/optimizer/mul_9_grad/Mul_1
�
Coptimizer/gradients_1/optimizer/mul_9_grad/tuple/control_dependencyIdentity.optimizer/gradients_1/optimizer/mul_9_grad/Mul<^optimizer/gradients_1/optimizer/mul_9_grad/tuple/group_deps*
T0*A
_class7
53loc:@optimizer/gradients_1/optimizer/mul_9_grad/Mul
�
Eoptimizer/gradients_1/optimizer/mul_9_grad/tuple/control_dependency_1Identity0optimizer/gradients_1/optimizer/mul_9_grad/Mul_1<^optimizer/gradients_1/optimizer/mul_9_grad/tuple/group_deps*
T0*C
_class9
75loc:@optimizer/gradients_1/optimizer/mul_9_grad/Mul_1
g
9optimizer/gradients_1/optimizer/Mean_7_grad/Reshape/shapeConst*
dtype0*
valueB:
�
3optimizer/gradients_1/optimizer/Mean_7_grad/ReshapeReshapeFoptimizer/gradients_1/optimizer/mul_10_grad/tuple/control_dependency_19optimizer/gradients_1/optimizer/Mean_7_grad/Reshape/shape*
T0*
Tshape0
s
1optimizer/gradients_1/optimizer/Mean_7_grad/ShapeShapeoptimizer/DynamicPartition_5:1*
T0*
out_type0
�
0optimizer/gradients_1/optimizer/Mean_7_grad/TileTile3optimizer/gradients_1/optimizer/Mean_7_grad/Reshape1optimizer/gradients_1/optimizer/Mean_7_grad/Shape*
T0*

Tmultiples0
u
3optimizer/gradients_1/optimizer/Mean_7_grad/Shape_1Shapeoptimizer/DynamicPartition_5:1*
T0*
out_type0
\
3optimizer/gradients_1/optimizer/Mean_7_grad/Shape_2Const*
dtype0*
valueB 
_
1optimizer/gradients_1/optimizer/Mean_7_grad/ConstConst*
dtype0*
valueB: 
�
0optimizer/gradients_1/optimizer/Mean_7_grad/ProdProd3optimizer/gradients_1/optimizer/Mean_7_grad/Shape_11optimizer/gradients_1/optimizer/Mean_7_grad/Const*
T0*

Tidx0*
	keep_dims( 
a
3optimizer/gradients_1/optimizer/Mean_7_grad/Const_1Const*
dtype0*
valueB: 
�
2optimizer/gradients_1/optimizer/Mean_7_grad/Prod_1Prod3optimizer/gradients_1/optimizer/Mean_7_grad/Shape_23optimizer/gradients_1/optimizer/Mean_7_grad/Const_1*
T0*

Tidx0*
	keep_dims( 
_
5optimizer/gradients_1/optimizer/Mean_7_grad/Maximum/yConst*
dtype0*
value	B :
�
3optimizer/gradients_1/optimizer/Mean_7_grad/MaximumMaximum2optimizer/gradients_1/optimizer/Mean_7_grad/Prod_15optimizer/gradients_1/optimizer/Mean_7_grad/Maximum/y*
T0
�
4optimizer/gradients_1/optimizer/Mean_7_grad/floordivFloorDiv0optimizer/gradients_1/optimizer/Mean_7_grad/Prod3optimizer/gradients_1/optimizer/Mean_7_grad/Maximum*
T0
�
0optimizer/gradients_1/optimizer/Mean_7_grad/CastCast4optimizer/gradients_1/optimizer/Mean_7_grad/floordiv*

DstT0*

SrcT0*
Truncate( 
�
3optimizer/gradients_1/optimizer/Mean_7_grad/truedivRealDiv0optimizer/gradients_1/optimizer/Mean_7_grad/Tile0optimizer/gradients_1/optimizer/Mean_7_grad/Cast*
T0
n
9optimizer/gradients_1/optimizer/Mean_6_grad/Reshape/shapeConst*
dtype0*
valueB"      
�
3optimizer/gradients_1/optimizer/Mean_6_grad/ReshapeReshape.optimizer/gradients_1/optimizer/Neg_5_grad/Neg9optimizer/gradients_1/optimizer/Mean_6_grad/Reshape/shape*
T0*
Tshape0
s
1optimizer/gradients_1/optimizer/Mean_6_grad/ShapeShapeoptimizer/DynamicPartition_4:1*
T0*
out_type0
�
0optimizer/gradients_1/optimizer/Mean_6_grad/TileTile3optimizer/gradients_1/optimizer/Mean_6_grad/Reshape1optimizer/gradients_1/optimizer/Mean_6_grad/Shape*
T0*

Tmultiples0
u
3optimizer/gradients_1/optimizer/Mean_6_grad/Shape_1Shapeoptimizer/DynamicPartition_4:1*
T0*
out_type0
\
3optimizer/gradients_1/optimizer/Mean_6_grad/Shape_2Const*
dtype0*
valueB 
_
1optimizer/gradients_1/optimizer/Mean_6_grad/ConstConst*
dtype0*
valueB: 
�
0optimizer/gradients_1/optimizer/Mean_6_grad/ProdProd3optimizer/gradients_1/optimizer/Mean_6_grad/Shape_11optimizer/gradients_1/optimizer/Mean_6_grad/Const*
T0*

Tidx0*
	keep_dims( 
a
3optimizer/gradients_1/optimizer/Mean_6_grad/Const_1Const*
dtype0*
valueB: 
�
2optimizer/gradients_1/optimizer/Mean_6_grad/Prod_1Prod3optimizer/gradients_1/optimizer/Mean_6_grad/Shape_23optimizer/gradients_1/optimizer/Mean_6_grad/Const_1*
T0*

Tidx0*
	keep_dims( 
_
5optimizer/gradients_1/optimizer/Mean_6_grad/Maximum/yConst*
dtype0*
value	B :
�
3optimizer/gradients_1/optimizer/Mean_6_grad/MaximumMaximum2optimizer/gradients_1/optimizer/Mean_6_grad/Prod_15optimizer/gradients_1/optimizer/Mean_6_grad/Maximum/y*
T0
�
4optimizer/gradients_1/optimizer/Mean_6_grad/floordivFloorDiv0optimizer/gradients_1/optimizer/Mean_6_grad/Prod3optimizer/gradients_1/optimizer/Mean_6_grad/Maximum*
T0
�
0optimizer/gradients_1/optimizer/Mean_6_grad/CastCast4optimizer/gradients_1/optimizer/Mean_6_grad/floordiv*

DstT0*

SrcT0*
Truncate( 
�
3optimizer/gradients_1/optimizer/Mean_6_grad/truedivRealDiv0optimizer/gradients_1/optimizer/Mean_6_grad/Tile0optimizer/gradients_1/optimizer/Mean_6_grad/Cast*
T0
g
9optimizer/gradients_1/optimizer/Mean_5_grad/Reshape/shapeConst*
dtype0*
valueB:
�
3optimizer/gradients_1/optimizer/Mean_5_grad/ReshapeReshapeEoptimizer/gradients_1/optimizer/mul_9_grad/tuple/control_dependency_19optimizer/gradients_1/optimizer/Mean_5_grad/Reshape/shape*
T0*
Tshape0
_
1optimizer/gradients_1/optimizer/Mean_5_grad/ConstConst*
dtype0*
valueB:
�
0optimizer/gradients_1/optimizer/Mean_5_grad/TileTile3optimizer/gradients_1/optimizer/Mean_5_grad/Reshape1optimizer/gradients_1/optimizer/Mean_5_grad/Const*
T0*

Tmultiples0
`
3optimizer/gradients_1/optimizer/Mean_5_grad/Const_1Const*
dtype0*
valueB
 *   @
�
3optimizer/gradients_1/optimizer/Mean_5_grad/truedivRealDiv0optimizer/gradients_1/optimizer/Mean_5_grad/Tile3optimizer/gradients_1/optimizer/Mean_5_grad/Const_1*
T0
T
 optimizer/gradients_1/zeros_like	ZerosLikeoptimizer/DynamicPartition_5*
T0
e
=optimizer/gradients_1/optimizer/DynamicPartition_5_grad/ShapeShapeCast*
T0*
out_type0
k
=optimizer/gradients_1/optimizer/DynamicPartition_5_grad/ConstConst*
dtype0*
valueB: 
�
<optimizer/gradients_1/optimizer/DynamicPartition_5_grad/ProdProd=optimizer/gradients_1/optimizer/DynamicPartition_5_grad/Shape=optimizer/gradients_1/optimizer/DynamicPartition_5_grad/Const*
T0*

Tidx0*
	keep_dims( 
m
Coptimizer/gradients_1/optimizer/DynamicPartition_5_grad/range/startConst*
dtype0*
value	B : 
m
Coptimizer/gradients_1/optimizer/DynamicPartition_5_grad/range/deltaConst*
dtype0*
value	B :
�
=optimizer/gradients_1/optimizer/DynamicPartition_5_grad/rangeRangeCoptimizer/gradients_1/optimizer/DynamicPartition_5_grad/range/start<optimizer/gradients_1/optimizer/DynamicPartition_5_grad/ProdCoptimizer/gradients_1/optimizer/DynamicPartition_5_grad/range/delta*

Tidx0
�
?optimizer/gradients_1/optimizer/DynamicPartition_5_grad/ReshapeReshape=optimizer/gradients_1/optimizer/DynamicPartition_5_grad/range=optimizer/gradients_1/optimizer/DynamicPartition_5_grad/Shape*
T0*
Tshape0
�
Hoptimizer/gradients_1/optimizer/DynamicPartition_5_grad/DynamicPartitionDynamicPartition?optimizer/gradients_1/optimizer/DynamicPartition_5_grad/ReshapeCast*
T0*
num_partitions
�
Moptimizer/gradients_1/optimizer/DynamicPartition_5_grad/ParallelDynamicStitchParallelDynamicStitchHoptimizer/gradients_1/optimizer/DynamicPartition_5_grad/DynamicPartitionJoptimizer/gradients_1/optimizer/DynamicPartition_5_grad/DynamicPartition:1 optimizer/gradients_1/zeros_like3optimizer/gradients_1/optimizer/Mean_7_grad/truediv*
N*
T0
q
?optimizer/gradients_1/optimizer/DynamicPartition_5_grad/Shape_1Shapepolicy_1/Sum_2*
T0*
out_type0
�
Aoptimizer/gradients_1/optimizer/DynamicPartition_5_grad/Reshape_1ReshapeMoptimizer/gradients_1/optimizer/DynamicPartition_5_grad/ParallelDynamicStitch?optimizer/gradients_1/optimizer/DynamicPartition_5_grad/Shape_1*
T0*
Tshape0
V
"optimizer/gradients_1/zeros_like_1	ZerosLikeoptimizer/DynamicPartition_4*
T0
e
=optimizer/gradients_1/optimizer/DynamicPartition_4_grad/ShapeShapeCast*
T0*
out_type0
k
=optimizer/gradients_1/optimizer/DynamicPartition_4_grad/ConstConst*
dtype0*
valueB: 
�
<optimizer/gradients_1/optimizer/DynamicPartition_4_grad/ProdProd=optimizer/gradients_1/optimizer/DynamicPartition_4_grad/Shape=optimizer/gradients_1/optimizer/DynamicPartition_4_grad/Const*
T0*

Tidx0*
	keep_dims( 
m
Coptimizer/gradients_1/optimizer/DynamicPartition_4_grad/range/startConst*
dtype0*
value	B : 
m
Coptimizer/gradients_1/optimizer/DynamicPartition_4_grad/range/deltaConst*
dtype0*
value	B :
�
=optimizer/gradients_1/optimizer/DynamicPartition_4_grad/rangeRangeCoptimizer/gradients_1/optimizer/DynamicPartition_4_grad/range/start<optimizer/gradients_1/optimizer/DynamicPartition_4_grad/ProdCoptimizer/gradients_1/optimizer/DynamicPartition_4_grad/range/delta*

Tidx0
�
?optimizer/gradients_1/optimizer/DynamicPartition_4_grad/ReshapeReshape=optimizer/gradients_1/optimizer/DynamicPartition_4_grad/range=optimizer/gradients_1/optimizer/DynamicPartition_4_grad/Shape*
T0*
Tshape0
�
Hoptimizer/gradients_1/optimizer/DynamicPartition_4_grad/DynamicPartitionDynamicPartition?optimizer/gradients_1/optimizer/DynamicPartition_4_grad/ReshapeCast*
T0*
num_partitions
�
Moptimizer/gradients_1/optimizer/DynamicPartition_4_grad/ParallelDynamicStitchParallelDynamicStitchHoptimizer/gradients_1/optimizer/DynamicPartition_4_grad/DynamicPartitionJoptimizer/gradients_1/optimizer/DynamicPartition_4_grad/DynamicPartition:1"optimizer/gradients_1/zeros_like_13optimizer/gradients_1/optimizer/Mean_6_grad/truediv*
N*
T0
t
?optimizer/gradients_1/optimizer/DynamicPartition_4_grad/Shape_1Shapeoptimizer/Minimum*
T0*
out_type0
�
Aoptimizer/gradients_1/optimizer/DynamicPartition_4_grad/Reshape_1ReshapeMoptimizer/gradients_1/optimizer/DynamicPartition_4_grad/ParallelDynamicStitch?optimizer/gradients_1/optimizer/DynamicPartition_4_grad/Shape_1*
T0*
Tshape0
�
9optimizer/gradients_1/optimizer/Mean_5/input_grad/unstackUnpack3optimizer/gradients_1/optimizer/Mean_5_grad/truediv*
T0*

axis *	
num
�
Boptimizer/gradients_1/optimizer/Mean_5/input_grad/tuple/group_depsNoOp:^optimizer/gradients_1/optimizer/Mean_5/input_grad/unstack
�
Joptimizer/gradients_1/optimizer/Mean_5/input_grad/tuple/control_dependencyIdentity9optimizer/gradients_1/optimizer/Mean_5/input_grad/unstackC^optimizer/gradients_1/optimizer/Mean_5/input_grad/tuple/group_deps*
T0*L
_classB
@>loc:@optimizer/gradients_1/optimizer/Mean_5/input_grad/unstack
�
Loptimizer/gradients_1/optimizer/Mean_5/input_grad/tuple/control_dependency_1Identity;optimizer/gradients_1/optimizer/Mean_5/input_grad/unstack:1C^optimizer/gradients_1/optimizer/Mean_5/input_grad/tuple/group_deps*
T0*L
_classB
@>loc:@optimizer/gradients_1/optimizer/Mean_5/input_grad/unstack
a
/optimizer/gradients_1/policy_1/Sum_2_grad/ShapeShapepolicy_1/stack*
T0*
out_type0
�
.optimizer/gradients_1/policy_1/Sum_2_grad/SizeConst*B
_class8
64loc:@optimizer/gradients_1/policy_1/Sum_2_grad/Shape*
dtype0*
value	B :
�
-optimizer/gradients_1/policy_1/Sum_2_grad/addAddV2 policy_1/Sum_2/reduction_indices.optimizer/gradients_1/policy_1/Sum_2_grad/Size*
T0*B
_class8
64loc:@optimizer/gradients_1/policy_1/Sum_2_grad/Shape
�
-optimizer/gradients_1/policy_1/Sum_2_grad/modFloorMod-optimizer/gradients_1/policy_1/Sum_2_grad/add.optimizer/gradients_1/policy_1/Sum_2_grad/Size*
T0*B
_class8
64loc:@optimizer/gradients_1/policy_1/Sum_2_grad/Shape
�
1optimizer/gradients_1/policy_1/Sum_2_grad/Shape_1Const*B
_class8
64loc:@optimizer/gradients_1/policy_1/Sum_2_grad/Shape*
dtype0*
valueB 
�
5optimizer/gradients_1/policy_1/Sum_2_grad/range/startConst*B
_class8
64loc:@optimizer/gradients_1/policy_1/Sum_2_grad/Shape*
dtype0*
value	B : 
�
5optimizer/gradients_1/policy_1/Sum_2_grad/range/deltaConst*B
_class8
64loc:@optimizer/gradients_1/policy_1/Sum_2_grad/Shape*
dtype0*
value	B :
�
/optimizer/gradients_1/policy_1/Sum_2_grad/rangeRange5optimizer/gradients_1/policy_1/Sum_2_grad/range/start.optimizer/gradients_1/policy_1/Sum_2_grad/Size5optimizer/gradients_1/policy_1/Sum_2_grad/range/delta*

Tidx0*B
_class8
64loc:@optimizer/gradients_1/policy_1/Sum_2_grad/Shape
�
4optimizer/gradients_1/policy_1/Sum_2_grad/Fill/valueConst*B
_class8
64loc:@optimizer/gradients_1/policy_1/Sum_2_grad/Shape*
dtype0*
value	B :
�
.optimizer/gradients_1/policy_1/Sum_2_grad/FillFill1optimizer/gradients_1/policy_1/Sum_2_grad/Shape_14optimizer/gradients_1/policy_1/Sum_2_grad/Fill/value*
T0*B
_class8
64loc:@optimizer/gradients_1/policy_1/Sum_2_grad/Shape*

index_type0
�
7optimizer/gradients_1/policy_1/Sum_2_grad/DynamicStitchDynamicStitch/optimizer/gradients_1/policy_1/Sum_2_grad/range-optimizer/gradients_1/policy_1/Sum_2_grad/mod/optimizer/gradients_1/policy_1/Sum_2_grad/Shape.optimizer/gradients_1/policy_1/Sum_2_grad/Fill*
N*
T0*B
_class8
64loc:@optimizer/gradients_1/policy_1/Sum_2_grad/Shape
�
1optimizer/gradients_1/policy_1/Sum_2_grad/ReshapeReshapeAoptimizer/gradients_1/optimizer/DynamicPartition_5_grad/Reshape_17optimizer/gradients_1/policy_1/Sum_2_grad/DynamicStitch*
T0*
Tshape0
�
5optimizer/gradients_1/policy_1/Sum_2_grad/BroadcastToBroadcastTo1optimizer/gradients_1/policy_1/Sum_2_grad/Reshape/optimizer/gradients_1/policy_1/Sum_2_grad/Shape*
T0*

Tidx0
e
2optimizer/gradients_1/optimizer/Minimum_grad/ShapeShapeoptimizer/mul_7*
T0*
out_type0
g
4optimizer/gradients_1/optimizer/Minimum_grad/Shape_1Shapeoptimizer/mul_8*
T0*
out_type0
�
4optimizer/gradients_1/optimizer/Minimum_grad/Shape_2ShapeAoptimizer/gradients_1/optimizer/DynamicPartition_4_grad/Reshape_1*
T0*
out_type0
e
8optimizer/gradients_1/optimizer/Minimum_grad/zeros/ConstConst*
dtype0*
valueB
 *    
�
2optimizer/gradients_1/optimizer/Minimum_grad/zerosFill4optimizer/gradients_1/optimizer/Minimum_grad/Shape_28optimizer/gradients_1/optimizer/Minimum_grad/zeros/Const*
T0*

index_type0
n
6optimizer/gradients_1/optimizer/Minimum_grad/LessEqual	LessEqualoptimizer/mul_7optimizer/mul_8*
T0
�
Boptimizer/gradients_1/optimizer/Minimum_grad/BroadcastGradientArgsBroadcastGradientArgs2optimizer/gradients_1/optimizer/Minimum_grad/Shape4optimizer/gradients_1/optimizer/Minimum_grad/Shape_1*
T0
�
5optimizer/gradients_1/optimizer/Minimum_grad/SelectV2SelectV26optimizer/gradients_1/optimizer/Minimum_grad/LessEqualAoptimizer/gradients_1/optimizer/DynamicPartition_4_grad/Reshape_12optimizer/gradients_1/optimizer/Minimum_grad/zeros*
T0
�
0optimizer/gradients_1/optimizer/Minimum_grad/SumSum5optimizer/gradients_1/optimizer/Minimum_grad/SelectV2Boptimizer/gradients_1/optimizer/Minimum_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
4optimizer/gradients_1/optimizer/Minimum_grad/ReshapeReshape0optimizer/gradients_1/optimizer/Minimum_grad/Sum2optimizer/gradients_1/optimizer/Minimum_grad/Shape*
T0*
Tshape0
�
7optimizer/gradients_1/optimizer/Minimum_grad/SelectV2_1SelectV26optimizer/gradients_1/optimizer/Minimum_grad/LessEqual2optimizer/gradients_1/optimizer/Minimum_grad/zerosAoptimizer/gradients_1/optimizer/DynamicPartition_4_grad/Reshape_1*
T0
�
2optimizer/gradients_1/optimizer/Minimum_grad/Sum_1Sum7optimizer/gradients_1/optimizer/Minimum_grad/SelectV2_1Doptimizer/gradients_1/optimizer/Minimum_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
6optimizer/gradients_1/optimizer/Minimum_grad/Reshape_1Reshape2optimizer/gradients_1/optimizer/Minimum_grad/Sum_14optimizer/gradients_1/optimizer/Minimum_grad/Shape_1*
T0*
Tshape0
�
=optimizer/gradients_1/optimizer/Minimum_grad/tuple/group_depsNoOp5^optimizer/gradients_1/optimizer/Minimum_grad/Reshape7^optimizer/gradients_1/optimizer/Minimum_grad/Reshape_1
�
Eoptimizer/gradients_1/optimizer/Minimum_grad/tuple/control_dependencyIdentity4optimizer/gradients_1/optimizer/Minimum_grad/Reshape>^optimizer/gradients_1/optimizer/Minimum_grad/tuple/group_deps*
T0*G
_class=
;9loc:@optimizer/gradients_1/optimizer/Minimum_grad/Reshape
�
Goptimizer/gradients_1/optimizer/Minimum_grad/tuple/control_dependency_1Identity6optimizer/gradients_1/optimizer/Minimum_grad/Reshape_1>^optimizer/gradients_1/optimizer/Minimum_grad/tuple/group_deps*
T0*I
_class?
=;loc:@optimizer/gradients_1/optimizer/Minimum_grad/Reshape_1
g
9optimizer/gradients_1/optimizer/Mean_3_grad/Reshape/shapeConst*
dtype0*
valueB:
�
3optimizer/gradients_1/optimizer/Mean_3_grad/ReshapeReshapeJoptimizer/gradients_1/optimizer/Mean_5/input_grad/tuple/control_dependency9optimizer/gradients_1/optimizer/Mean_3_grad/Reshape/shape*
T0*
Tshape0
s
1optimizer/gradients_1/optimizer/Mean_3_grad/ShapeShapeoptimizer/DynamicPartition_2:1*
T0*
out_type0
�
0optimizer/gradients_1/optimizer/Mean_3_grad/TileTile3optimizer/gradients_1/optimizer/Mean_3_grad/Reshape1optimizer/gradients_1/optimizer/Mean_3_grad/Shape*
T0*

Tmultiples0
u
3optimizer/gradients_1/optimizer/Mean_3_grad/Shape_1Shapeoptimizer/DynamicPartition_2:1*
T0*
out_type0
\
3optimizer/gradients_1/optimizer/Mean_3_grad/Shape_2Const*
dtype0*
valueB 
_
1optimizer/gradients_1/optimizer/Mean_3_grad/ConstConst*
dtype0*
valueB: 
�
0optimizer/gradients_1/optimizer/Mean_3_grad/ProdProd3optimizer/gradients_1/optimizer/Mean_3_grad/Shape_11optimizer/gradients_1/optimizer/Mean_3_grad/Const*
T0*

Tidx0*
	keep_dims( 
a
3optimizer/gradients_1/optimizer/Mean_3_grad/Const_1Const*
dtype0*
valueB: 
�
2optimizer/gradients_1/optimizer/Mean_3_grad/Prod_1Prod3optimizer/gradients_1/optimizer/Mean_3_grad/Shape_23optimizer/gradients_1/optimizer/Mean_3_grad/Const_1*
T0*

Tidx0*
	keep_dims( 
_
5optimizer/gradients_1/optimizer/Mean_3_grad/Maximum/yConst*
dtype0*
value	B :
�
3optimizer/gradients_1/optimizer/Mean_3_grad/MaximumMaximum2optimizer/gradients_1/optimizer/Mean_3_grad/Prod_15optimizer/gradients_1/optimizer/Mean_3_grad/Maximum/y*
T0
�
4optimizer/gradients_1/optimizer/Mean_3_grad/floordivFloorDiv0optimizer/gradients_1/optimizer/Mean_3_grad/Prod3optimizer/gradients_1/optimizer/Mean_3_grad/Maximum*
T0
�
0optimizer/gradients_1/optimizer/Mean_3_grad/CastCast4optimizer/gradients_1/optimizer/Mean_3_grad/floordiv*

DstT0*

SrcT0*
Truncate( 
�
3optimizer/gradients_1/optimizer/Mean_3_grad/truedivRealDiv0optimizer/gradients_1/optimizer/Mean_3_grad/Tile0optimizer/gradients_1/optimizer/Mean_3_grad/Cast*
T0
g
9optimizer/gradients_1/optimizer/Mean_4_grad/Reshape/shapeConst*
dtype0*
valueB:
�
3optimizer/gradients_1/optimizer/Mean_4_grad/ReshapeReshapeLoptimizer/gradients_1/optimizer/Mean_5/input_grad/tuple/control_dependency_19optimizer/gradients_1/optimizer/Mean_4_grad/Reshape/shape*
T0*
Tshape0
s
1optimizer/gradients_1/optimizer/Mean_4_grad/ShapeShapeoptimizer/DynamicPartition_3:1*
T0*
out_type0
�
0optimizer/gradients_1/optimizer/Mean_4_grad/TileTile3optimizer/gradients_1/optimizer/Mean_4_grad/Reshape1optimizer/gradients_1/optimizer/Mean_4_grad/Shape*
T0*

Tmultiples0
u
3optimizer/gradients_1/optimizer/Mean_4_grad/Shape_1Shapeoptimizer/DynamicPartition_3:1*
T0*
out_type0
\
3optimizer/gradients_1/optimizer/Mean_4_grad/Shape_2Const*
dtype0*
valueB 
_
1optimizer/gradients_1/optimizer/Mean_4_grad/ConstConst*
dtype0*
valueB: 
�
0optimizer/gradients_1/optimizer/Mean_4_grad/ProdProd3optimizer/gradients_1/optimizer/Mean_4_grad/Shape_11optimizer/gradients_1/optimizer/Mean_4_grad/Const*
T0*

Tidx0*
	keep_dims( 
a
3optimizer/gradients_1/optimizer/Mean_4_grad/Const_1Const*
dtype0*
valueB: 
�
2optimizer/gradients_1/optimizer/Mean_4_grad/Prod_1Prod3optimizer/gradients_1/optimizer/Mean_4_grad/Shape_23optimizer/gradients_1/optimizer/Mean_4_grad/Const_1*
T0*

Tidx0*
	keep_dims( 
_
5optimizer/gradients_1/optimizer/Mean_4_grad/Maximum/yConst*
dtype0*
value	B :
�
3optimizer/gradients_1/optimizer/Mean_4_grad/MaximumMaximum2optimizer/gradients_1/optimizer/Mean_4_grad/Prod_15optimizer/gradients_1/optimizer/Mean_4_grad/Maximum/y*
T0
�
4optimizer/gradients_1/optimizer/Mean_4_grad/floordivFloorDiv0optimizer/gradients_1/optimizer/Mean_4_grad/Prod3optimizer/gradients_1/optimizer/Mean_4_grad/Maximum*
T0
�
0optimizer/gradients_1/optimizer/Mean_4_grad/CastCast4optimizer/gradients_1/optimizer/Mean_4_grad/floordiv*

DstT0*

SrcT0*
Truncate( 
�
3optimizer/gradients_1/optimizer/Mean_4_grad/truedivRealDiv0optimizer/gradients_1/optimizer/Mean_4_grad/Tile0optimizer/gradients_1/optimizer/Mean_4_grad/Cast*
T0
�
1optimizer/gradients_1/policy_1/stack_grad/unstackUnpack5optimizer/gradients_1/policy_1/Sum_2_grad/BroadcastTo*
T0*

axis*	
num
v
:optimizer/gradients_1/policy_1/stack_grad/tuple/group_depsNoOp2^optimizer/gradients_1/policy_1/stack_grad/unstack
�
Boptimizer/gradients_1/policy_1/stack_grad/tuple/control_dependencyIdentity1optimizer/gradients_1/policy_1/stack_grad/unstack;^optimizer/gradients_1/policy_1/stack_grad/tuple/group_deps*
T0*D
_class:
86loc:@optimizer/gradients_1/policy_1/stack_grad/unstack
�
Doptimizer/gradients_1/policy_1/stack_grad/tuple/control_dependency_1Identity3optimizer/gradients_1/policy_1/stack_grad/unstack:1;^optimizer/gradients_1/policy_1/stack_grad/tuple/group_deps*
T0*D
_class:
86loc:@optimizer/gradients_1/policy_1/stack_grad/unstack
a
0optimizer/gradients_1/optimizer/mul_7_grad/ShapeShapeoptimizer/Exp*
T0*
out_type0
j
2optimizer/gradients_1/optimizer/mul_7_grad/Shape_1Shapeoptimizer/ExpandDims*
T0*
out_type0
�
@optimizer/gradients_1/optimizer/mul_7_grad/BroadcastGradientArgsBroadcastGradientArgs0optimizer/gradients_1/optimizer/mul_7_grad/Shape2optimizer/gradients_1/optimizer/mul_7_grad/Shape_1*
T0
�
.optimizer/gradients_1/optimizer/mul_7_grad/MulMulEoptimizer/gradients_1/optimizer/Minimum_grad/tuple/control_dependencyoptimizer/ExpandDims*
T0
�
.optimizer/gradients_1/optimizer/mul_7_grad/SumSum.optimizer/gradients_1/optimizer/mul_7_grad/Mul@optimizer/gradients_1/optimizer/mul_7_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
2optimizer/gradients_1/optimizer/mul_7_grad/ReshapeReshape.optimizer/gradients_1/optimizer/mul_7_grad/Sum0optimizer/gradients_1/optimizer/mul_7_grad/Shape*
T0*
Tshape0
�
0optimizer/gradients_1/optimizer/mul_7_grad/Mul_1Muloptimizer/ExpEoptimizer/gradients_1/optimizer/Minimum_grad/tuple/control_dependency*
T0
�
0optimizer/gradients_1/optimizer/mul_7_grad/Sum_1Sum0optimizer/gradients_1/optimizer/mul_7_grad/Mul_1Boptimizer/gradients_1/optimizer/mul_7_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
4optimizer/gradients_1/optimizer/mul_7_grad/Reshape_1Reshape0optimizer/gradients_1/optimizer/mul_7_grad/Sum_12optimizer/gradients_1/optimizer/mul_7_grad/Shape_1*
T0*
Tshape0
�
;optimizer/gradients_1/optimizer/mul_7_grad/tuple/group_depsNoOp3^optimizer/gradients_1/optimizer/mul_7_grad/Reshape5^optimizer/gradients_1/optimizer/mul_7_grad/Reshape_1
�
Coptimizer/gradients_1/optimizer/mul_7_grad/tuple/control_dependencyIdentity2optimizer/gradients_1/optimizer/mul_7_grad/Reshape<^optimizer/gradients_1/optimizer/mul_7_grad/tuple/group_deps*
T0*E
_class;
97loc:@optimizer/gradients_1/optimizer/mul_7_grad/Reshape
�
Eoptimizer/gradients_1/optimizer/mul_7_grad/tuple/control_dependency_1Identity4optimizer/gradients_1/optimizer/mul_7_grad/Reshape_1<^optimizer/gradients_1/optimizer/mul_7_grad/tuple/group_deps*
T0*G
_class=
;9loc:@optimizer/gradients_1/optimizer/mul_7_grad/Reshape_1
m
0optimizer/gradients_1/optimizer/mul_8_grad/ShapeShapeoptimizer/clip_by_value_2*
T0*
out_type0
j
2optimizer/gradients_1/optimizer/mul_8_grad/Shape_1Shapeoptimizer/ExpandDims*
T0*
out_type0
�
@optimizer/gradients_1/optimizer/mul_8_grad/BroadcastGradientArgsBroadcastGradientArgs0optimizer/gradients_1/optimizer/mul_8_grad/Shape2optimizer/gradients_1/optimizer/mul_8_grad/Shape_1*
T0
�
.optimizer/gradients_1/optimizer/mul_8_grad/MulMulGoptimizer/gradients_1/optimizer/Minimum_grad/tuple/control_dependency_1optimizer/ExpandDims*
T0
�
.optimizer/gradients_1/optimizer/mul_8_grad/SumSum.optimizer/gradients_1/optimizer/mul_8_grad/Mul@optimizer/gradients_1/optimizer/mul_8_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
2optimizer/gradients_1/optimizer/mul_8_grad/ReshapeReshape.optimizer/gradients_1/optimizer/mul_8_grad/Sum0optimizer/gradients_1/optimizer/mul_8_grad/Shape*
T0*
Tshape0
�
0optimizer/gradients_1/optimizer/mul_8_grad/Mul_1Muloptimizer/clip_by_value_2Goptimizer/gradients_1/optimizer/Minimum_grad/tuple/control_dependency_1*
T0
�
0optimizer/gradients_1/optimizer/mul_8_grad/Sum_1Sum0optimizer/gradients_1/optimizer/mul_8_grad/Mul_1Boptimizer/gradients_1/optimizer/mul_8_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
4optimizer/gradients_1/optimizer/mul_8_grad/Reshape_1Reshape0optimizer/gradients_1/optimizer/mul_8_grad/Sum_12optimizer/gradients_1/optimizer/mul_8_grad/Shape_1*
T0*
Tshape0
�
;optimizer/gradients_1/optimizer/mul_8_grad/tuple/group_depsNoOp3^optimizer/gradients_1/optimizer/mul_8_grad/Reshape5^optimizer/gradients_1/optimizer/mul_8_grad/Reshape_1
�
Coptimizer/gradients_1/optimizer/mul_8_grad/tuple/control_dependencyIdentity2optimizer/gradients_1/optimizer/mul_8_grad/Reshape<^optimizer/gradients_1/optimizer/mul_8_grad/tuple/group_deps*
T0*E
_class;
97loc:@optimizer/gradients_1/optimizer/mul_8_grad/Reshape
�
Eoptimizer/gradients_1/optimizer/mul_8_grad/tuple/control_dependency_1Identity4optimizer/gradients_1/optimizer/mul_8_grad/Reshape_1<^optimizer/gradients_1/optimizer/mul_8_grad/tuple/group_deps*
T0*G
_class=
;9loc:@optimizer/gradients_1/optimizer/mul_8_grad/Reshape_1
V
"optimizer/gradients_1/zeros_like_2	ZerosLikeoptimizer/DynamicPartition_2*
T0
e
=optimizer/gradients_1/optimizer/DynamicPartition_2_grad/ShapeShapeCast*
T0*
out_type0
k
=optimizer/gradients_1/optimizer/DynamicPartition_2_grad/ConstConst*
dtype0*
valueB: 
�
<optimizer/gradients_1/optimizer/DynamicPartition_2_grad/ProdProd=optimizer/gradients_1/optimizer/DynamicPartition_2_grad/Shape=optimizer/gradients_1/optimizer/DynamicPartition_2_grad/Const*
T0*

Tidx0*
	keep_dims( 
m
Coptimizer/gradients_1/optimizer/DynamicPartition_2_grad/range/startConst*
dtype0*
value	B : 
m
Coptimizer/gradients_1/optimizer/DynamicPartition_2_grad/range/deltaConst*
dtype0*
value	B :
�
=optimizer/gradients_1/optimizer/DynamicPartition_2_grad/rangeRangeCoptimizer/gradients_1/optimizer/DynamicPartition_2_grad/range/start<optimizer/gradients_1/optimizer/DynamicPartition_2_grad/ProdCoptimizer/gradients_1/optimizer/DynamicPartition_2_grad/range/delta*

Tidx0
�
?optimizer/gradients_1/optimizer/DynamicPartition_2_grad/ReshapeReshape=optimizer/gradients_1/optimizer/DynamicPartition_2_grad/range=optimizer/gradients_1/optimizer/DynamicPartition_2_grad/Shape*
T0*
Tshape0
�
Hoptimizer/gradients_1/optimizer/DynamicPartition_2_grad/DynamicPartitionDynamicPartition?optimizer/gradients_1/optimizer/DynamicPartition_2_grad/ReshapeCast*
T0*
num_partitions
�
Moptimizer/gradients_1/optimizer/DynamicPartition_2_grad/ParallelDynamicStitchParallelDynamicStitchHoptimizer/gradients_1/optimizer/DynamicPartition_2_grad/DynamicPartitionJoptimizer/gradients_1/optimizer/DynamicPartition_2_grad/DynamicPartition:1"optimizer/gradients_1/zeros_like_23optimizer/gradients_1/optimizer/Mean_3_grad/truediv*
N*
T0
t
?optimizer/gradients_1/optimizer/DynamicPartition_2_grad/Shape_1Shapeoptimizer/Maximum*
T0*
out_type0
�
Aoptimizer/gradients_1/optimizer/DynamicPartition_2_grad/Reshape_1ReshapeMoptimizer/gradients_1/optimizer/DynamicPartition_2_grad/ParallelDynamicStitch?optimizer/gradients_1/optimizer/DynamicPartition_2_grad/Shape_1*
T0*
Tshape0
V
"optimizer/gradients_1/zeros_like_3	ZerosLikeoptimizer/DynamicPartition_3*
T0
e
=optimizer/gradients_1/optimizer/DynamicPartition_3_grad/ShapeShapeCast*
T0*
out_type0
k
=optimizer/gradients_1/optimizer/DynamicPartition_3_grad/ConstConst*
dtype0*
valueB: 
�
<optimizer/gradients_1/optimizer/DynamicPartition_3_grad/ProdProd=optimizer/gradients_1/optimizer/DynamicPartition_3_grad/Shape=optimizer/gradients_1/optimizer/DynamicPartition_3_grad/Const*
T0*

Tidx0*
	keep_dims( 
m
Coptimizer/gradients_1/optimizer/DynamicPartition_3_grad/range/startConst*
dtype0*
value	B : 
m
Coptimizer/gradients_1/optimizer/DynamicPartition_3_grad/range/deltaConst*
dtype0*
value	B :
�
=optimizer/gradients_1/optimizer/DynamicPartition_3_grad/rangeRangeCoptimizer/gradients_1/optimizer/DynamicPartition_3_grad/range/start<optimizer/gradients_1/optimizer/DynamicPartition_3_grad/ProdCoptimizer/gradients_1/optimizer/DynamicPartition_3_grad/range/delta*

Tidx0
�
?optimizer/gradients_1/optimizer/DynamicPartition_3_grad/ReshapeReshape=optimizer/gradients_1/optimizer/DynamicPartition_3_grad/range=optimizer/gradients_1/optimizer/DynamicPartition_3_grad/Shape*
T0*
Tshape0
�
Hoptimizer/gradients_1/optimizer/DynamicPartition_3_grad/DynamicPartitionDynamicPartition?optimizer/gradients_1/optimizer/DynamicPartition_3_grad/ReshapeCast*
T0*
num_partitions
�
Moptimizer/gradients_1/optimizer/DynamicPartition_3_grad/ParallelDynamicStitchParallelDynamicStitchHoptimizer/gradients_1/optimizer/DynamicPartition_3_grad/DynamicPartitionJoptimizer/gradients_1/optimizer/DynamicPartition_3_grad/DynamicPartition:1"optimizer/gradients_1/zeros_like_33optimizer/gradients_1/optimizer/Mean_4_grad/truediv*
N*
T0
v
?optimizer/gradients_1/optimizer/DynamicPartition_3_grad/Shape_1Shapeoptimizer/Maximum_1*
T0*
out_type0
�
Aoptimizer/gradients_1/optimizer/DynamicPartition_3_grad/Reshape_1ReshapeMoptimizer/gradients_1/optimizer/DynamicPartition_3_grad/ParallelDynamicStitch?optimizer/gradients_1/optimizer/DynamicPartition_3_grad/Shape_1*
T0*
Tshape0
�
Uoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits/Reshape_2_grad/ShapeShape*policy_1/softmax_cross_entropy_with_logits*
T0*
out_type0
�
Woptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits/Reshape_2_grad/ReshapeReshapeBoptimizer/gradients_1/policy_1/stack_grad/tuple/control_dependencyUoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits/Reshape_2_grad/Shape*
T0*
Tshape0
�
Woptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_1/Reshape_2_grad/ShapeShape,policy_1/softmax_cross_entropy_with_logits_1*
T0*
out_type0
�
Yoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_1/Reshape_2_grad/ReshapeReshapeDoptimizer/gradients_1/policy_1/stack_grad/tuple/control_dependency_1Woptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_1/Reshape_2_grad/Shape*
T0*
Tshape0

:optimizer/gradients_1/optimizer/clip_by_value_2_grad/ShapeShape!optimizer/clip_by_value_2/Minimum*
T0*
out_type0
e
<optimizer/gradients_1/optimizer/clip_by_value_2_grad/Shape_1Const*
dtype0*
valueB 
�
<optimizer/gradients_1/optimizer/clip_by_value_2_grad/Shape_2ShapeCoptimizer/gradients_1/optimizer/mul_8_grad/tuple/control_dependency*
T0*
out_type0
m
@optimizer/gradients_1/optimizer/clip_by_value_2_grad/zeros/ConstConst*
dtype0*
valueB
 *    
�
:optimizer/gradients_1/optimizer/clip_by_value_2_grad/zerosFill<optimizer/gradients_1/optimizer/clip_by_value_2_grad/Shape_2@optimizer/gradients_1/optimizer/clip_by_value_2_grad/zeros/Const*
T0*

index_type0
�
Aoptimizer/gradients_1/optimizer/clip_by_value_2_grad/GreaterEqualGreaterEqual!optimizer/clip_by_value_2/Minimumoptimizer/sub_3*
T0
�
Joptimizer/gradients_1/optimizer/clip_by_value_2_grad/BroadcastGradientArgsBroadcastGradientArgs:optimizer/gradients_1/optimizer/clip_by_value_2_grad/Shape<optimizer/gradients_1/optimizer/clip_by_value_2_grad/Shape_1*
T0
�
=optimizer/gradients_1/optimizer/clip_by_value_2_grad/SelectV2SelectV2Aoptimizer/gradients_1/optimizer/clip_by_value_2_grad/GreaterEqualCoptimizer/gradients_1/optimizer/mul_8_grad/tuple/control_dependency:optimizer/gradients_1/optimizer/clip_by_value_2_grad/zeros*
T0
�
8optimizer/gradients_1/optimizer/clip_by_value_2_grad/SumSum=optimizer/gradients_1/optimizer/clip_by_value_2_grad/SelectV2Joptimizer/gradients_1/optimizer/clip_by_value_2_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
<optimizer/gradients_1/optimizer/clip_by_value_2_grad/ReshapeReshape8optimizer/gradients_1/optimizer/clip_by_value_2_grad/Sum:optimizer/gradients_1/optimizer/clip_by_value_2_grad/Shape*
T0*
Tshape0
�
?optimizer/gradients_1/optimizer/clip_by_value_2_grad/SelectV2_1SelectV2Aoptimizer/gradients_1/optimizer/clip_by_value_2_grad/GreaterEqual:optimizer/gradients_1/optimizer/clip_by_value_2_grad/zerosCoptimizer/gradients_1/optimizer/mul_8_grad/tuple/control_dependency*
T0
�
:optimizer/gradients_1/optimizer/clip_by_value_2_grad/Sum_1Sum?optimizer/gradients_1/optimizer/clip_by_value_2_grad/SelectV2_1Loptimizer/gradients_1/optimizer/clip_by_value_2_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
>optimizer/gradients_1/optimizer/clip_by_value_2_grad/Reshape_1Reshape:optimizer/gradients_1/optimizer/clip_by_value_2_grad/Sum_1<optimizer/gradients_1/optimizer/clip_by_value_2_grad/Shape_1*
T0*
Tshape0
�
Eoptimizer/gradients_1/optimizer/clip_by_value_2_grad/tuple/group_depsNoOp=^optimizer/gradients_1/optimizer/clip_by_value_2_grad/Reshape?^optimizer/gradients_1/optimizer/clip_by_value_2_grad/Reshape_1
�
Moptimizer/gradients_1/optimizer/clip_by_value_2_grad/tuple/control_dependencyIdentity<optimizer/gradients_1/optimizer/clip_by_value_2_grad/ReshapeF^optimizer/gradients_1/optimizer/clip_by_value_2_grad/tuple/group_deps*
T0*O
_classE
CAloc:@optimizer/gradients_1/optimizer/clip_by_value_2_grad/Reshape
�
Ooptimizer/gradients_1/optimizer/clip_by_value_2_grad/tuple/control_dependency_1Identity>optimizer/gradients_1/optimizer/clip_by_value_2_grad/Reshape_1F^optimizer/gradients_1/optimizer/clip_by_value_2_grad/tuple/group_deps*
T0*Q
_classG
ECloc:@optimizer/gradients_1/optimizer/clip_by_value_2_grad/Reshape_1
s
2optimizer/gradients_1/optimizer/Maximum_grad/ShapeShapeoptimizer/SquaredDifference_1*
T0*
out_type0
u
4optimizer/gradients_1/optimizer/Maximum_grad/Shape_1Shapeoptimizer/SquaredDifference_2*
T0*
out_type0
�
4optimizer/gradients_1/optimizer/Maximum_grad/Shape_2ShapeAoptimizer/gradients_1/optimizer/DynamicPartition_2_grad/Reshape_1*
T0*
out_type0
e
8optimizer/gradients_1/optimizer/Maximum_grad/zeros/ConstConst*
dtype0*
valueB
 *    
�
2optimizer/gradients_1/optimizer/Maximum_grad/zerosFill4optimizer/gradients_1/optimizer/Maximum_grad/Shape_28optimizer/gradients_1/optimizer/Maximum_grad/zeros/Const*
T0*

index_type0
�
9optimizer/gradients_1/optimizer/Maximum_grad/GreaterEqualGreaterEqualoptimizer/SquaredDifference_1optimizer/SquaredDifference_2*
T0
�
Boptimizer/gradients_1/optimizer/Maximum_grad/BroadcastGradientArgsBroadcastGradientArgs2optimizer/gradients_1/optimizer/Maximum_grad/Shape4optimizer/gradients_1/optimizer/Maximum_grad/Shape_1*
T0
�
5optimizer/gradients_1/optimizer/Maximum_grad/SelectV2SelectV29optimizer/gradients_1/optimizer/Maximum_grad/GreaterEqualAoptimizer/gradients_1/optimizer/DynamicPartition_2_grad/Reshape_12optimizer/gradients_1/optimizer/Maximum_grad/zeros*
T0
�
0optimizer/gradients_1/optimizer/Maximum_grad/SumSum5optimizer/gradients_1/optimizer/Maximum_grad/SelectV2Boptimizer/gradients_1/optimizer/Maximum_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
4optimizer/gradients_1/optimizer/Maximum_grad/ReshapeReshape0optimizer/gradients_1/optimizer/Maximum_grad/Sum2optimizer/gradients_1/optimizer/Maximum_grad/Shape*
T0*
Tshape0
�
7optimizer/gradients_1/optimizer/Maximum_grad/SelectV2_1SelectV29optimizer/gradients_1/optimizer/Maximum_grad/GreaterEqual2optimizer/gradients_1/optimizer/Maximum_grad/zerosAoptimizer/gradients_1/optimizer/DynamicPartition_2_grad/Reshape_1*
T0
�
2optimizer/gradients_1/optimizer/Maximum_grad/Sum_1Sum7optimizer/gradients_1/optimizer/Maximum_grad/SelectV2_1Doptimizer/gradients_1/optimizer/Maximum_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
6optimizer/gradients_1/optimizer/Maximum_grad/Reshape_1Reshape2optimizer/gradients_1/optimizer/Maximum_grad/Sum_14optimizer/gradients_1/optimizer/Maximum_grad/Shape_1*
T0*
Tshape0
�
=optimizer/gradients_1/optimizer/Maximum_grad/tuple/group_depsNoOp5^optimizer/gradients_1/optimizer/Maximum_grad/Reshape7^optimizer/gradients_1/optimizer/Maximum_grad/Reshape_1
�
Eoptimizer/gradients_1/optimizer/Maximum_grad/tuple/control_dependencyIdentity4optimizer/gradients_1/optimizer/Maximum_grad/Reshape>^optimizer/gradients_1/optimizer/Maximum_grad/tuple/group_deps*
T0*G
_class=
;9loc:@optimizer/gradients_1/optimizer/Maximum_grad/Reshape
�
Goptimizer/gradients_1/optimizer/Maximum_grad/tuple/control_dependency_1Identity6optimizer/gradients_1/optimizer/Maximum_grad/Reshape_1>^optimizer/gradients_1/optimizer/Maximum_grad/tuple/group_deps*
T0*I
_class?
=;loc:@optimizer/gradients_1/optimizer/Maximum_grad/Reshape_1
u
4optimizer/gradients_1/optimizer/Maximum_1_grad/ShapeShapeoptimizer/SquaredDifference_3*
T0*
out_type0
w
6optimizer/gradients_1/optimizer/Maximum_1_grad/Shape_1Shapeoptimizer/SquaredDifference_4*
T0*
out_type0
�
6optimizer/gradients_1/optimizer/Maximum_1_grad/Shape_2ShapeAoptimizer/gradients_1/optimizer/DynamicPartition_3_grad/Reshape_1*
T0*
out_type0
g
:optimizer/gradients_1/optimizer/Maximum_1_grad/zeros/ConstConst*
dtype0*
valueB
 *    
�
4optimizer/gradients_1/optimizer/Maximum_1_grad/zerosFill6optimizer/gradients_1/optimizer/Maximum_1_grad/Shape_2:optimizer/gradients_1/optimizer/Maximum_1_grad/zeros/Const*
T0*

index_type0
�
;optimizer/gradients_1/optimizer/Maximum_1_grad/GreaterEqualGreaterEqualoptimizer/SquaredDifference_3optimizer/SquaredDifference_4*
T0
�
Doptimizer/gradients_1/optimizer/Maximum_1_grad/BroadcastGradientArgsBroadcastGradientArgs4optimizer/gradients_1/optimizer/Maximum_1_grad/Shape6optimizer/gradients_1/optimizer/Maximum_1_grad/Shape_1*
T0
�
7optimizer/gradients_1/optimizer/Maximum_1_grad/SelectV2SelectV2;optimizer/gradients_1/optimizer/Maximum_1_grad/GreaterEqualAoptimizer/gradients_1/optimizer/DynamicPartition_3_grad/Reshape_14optimizer/gradients_1/optimizer/Maximum_1_grad/zeros*
T0
�
2optimizer/gradients_1/optimizer/Maximum_1_grad/SumSum7optimizer/gradients_1/optimizer/Maximum_1_grad/SelectV2Doptimizer/gradients_1/optimizer/Maximum_1_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
6optimizer/gradients_1/optimizer/Maximum_1_grad/ReshapeReshape2optimizer/gradients_1/optimizer/Maximum_1_grad/Sum4optimizer/gradients_1/optimizer/Maximum_1_grad/Shape*
T0*
Tshape0
�
9optimizer/gradients_1/optimizer/Maximum_1_grad/SelectV2_1SelectV2;optimizer/gradients_1/optimizer/Maximum_1_grad/GreaterEqual4optimizer/gradients_1/optimizer/Maximum_1_grad/zerosAoptimizer/gradients_1/optimizer/DynamicPartition_3_grad/Reshape_1*
T0
�
4optimizer/gradients_1/optimizer/Maximum_1_grad/Sum_1Sum9optimizer/gradients_1/optimizer/Maximum_1_grad/SelectV2_1Foptimizer/gradients_1/optimizer/Maximum_1_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
8optimizer/gradients_1/optimizer/Maximum_1_grad/Reshape_1Reshape4optimizer/gradients_1/optimizer/Maximum_1_grad/Sum_16optimizer/gradients_1/optimizer/Maximum_1_grad/Shape_1*
T0*
Tshape0
�
?optimizer/gradients_1/optimizer/Maximum_1_grad/tuple/group_depsNoOp7^optimizer/gradients_1/optimizer/Maximum_1_grad/Reshape9^optimizer/gradients_1/optimizer/Maximum_1_grad/Reshape_1
�
Goptimizer/gradients_1/optimizer/Maximum_1_grad/tuple/control_dependencyIdentity6optimizer/gradients_1/optimizer/Maximum_1_grad/Reshape@^optimizer/gradients_1/optimizer/Maximum_1_grad/tuple/group_deps*
T0*I
_class?
=;loc:@optimizer/gradients_1/optimizer/Maximum_1_grad/Reshape
�
Ioptimizer/gradients_1/optimizer/Maximum_1_grad/tuple/control_dependency_1Identity8optimizer/gradients_1/optimizer/Maximum_1_grad/Reshape_1@^optimizer/gradients_1/optimizer/Maximum_1_grad/tuple/group_deps*
T0*K
_classA
?=loc:@optimizer/gradients_1/optimizer/Maximum_1_grad/Reshape_1
f
"optimizer/gradients_1/zeros_like_4	ZerosLike,policy_1/softmax_cross_entropy_with_logits:1*
T0
�
Toptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_grad/ExpandDims/dimConst*
dtype0*
valueB :
���������
�
Poptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_grad/ExpandDims
ExpandDimsWoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits/Reshape_2_grad/ReshapeToptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_grad/ExpandDims/dim*
T0*

Tdim0
�
Ioptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_grad/mulMulPoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_grad/ExpandDims,policy_1/softmax_cross_entropy_with_logits:1*
T0
�
Poptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_grad/LogSoftmax
LogSoftmax2policy_1/softmax_cross_entropy_with_logits/Reshape*
T0
�
Ioptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_grad/NegNegPoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_grad/LogSoftmax*
T0
�
Voptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_grad/ExpandDims_1/dimConst*
dtype0*
valueB :
���������
�
Roptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_grad/ExpandDims_1
ExpandDimsWoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits/Reshape_2_grad/ReshapeVoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_grad/ExpandDims_1/dim*
T0*

Tdim0
�
Koptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_grad/mul_1MulRoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_grad/ExpandDims_1Ioptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_grad/Neg*
T0
�
Voptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_grad/tuple/group_depsNoOpJ^optimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_grad/mulL^optimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_grad/mul_1
�
^optimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_grad/tuple/control_dependencyIdentityIoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_grad/mulW^optimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_grad/tuple/group_deps*
T0*\
_classR
PNloc:@optimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_grad/mul
�
`optimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_grad/tuple/control_dependency_1IdentityKoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_grad/mul_1W^optimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_grad/tuple/group_deps*
T0*^
_classT
RPloc:@optimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_grad/mul_1
h
"optimizer/gradients_1/zeros_like_5	ZerosLike.policy_1/softmax_cross_entropy_with_logits_1:1*
T0
�
Voptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_1_grad/ExpandDims/dimConst*
dtype0*
valueB :
���������
�
Roptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_1_grad/ExpandDims
ExpandDimsYoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_1/Reshape_2_grad/ReshapeVoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_1_grad/ExpandDims/dim*
T0*

Tdim0
�
Koptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_1_grad/mulMulRoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_1_grad/ExpandDims.policy_1/softmax_cross_entropy_with_logits_1:1*
T0
�
Roptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_1_grad/LogSoftmax
LogSoftmax4policy_1/softmax_cross_entropy_with_logits_1/Reshape*
T0
�
Koptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_1_grad/NegNegRoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_1_grad/LogSoftmax*
T0
�
Xoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_1_grad/ExpandDims_1/dimConst*
dtype0*
valueB :
���������
�
Toptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_1_grad/ExpandDims_1
ExpandDimsYoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_1/Reshape_2_grad/ReshapeXoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_1_grad/ExpandDims_1/dim*
T0*

Tdim0
�
Moptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_1_grad/mul_1MulToptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_1_grad/ExpandDims_1Koptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_1_grad/Neg*
T0
�
Xoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_1_grad/tuple/group_depsNoOpL^optimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_1_grad/mulN^optimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_1_grad/mul_1
�
`optimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_1_grad/tuple/control_dependencyIdentityKoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_1_grad/mulY^optimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_1_grad/tuple/group_deps*
T0*^
_classT
RPloc:@optimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_1_grad/mul
�
boptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_1_grad/tuple/control_dependency_1IdentityMoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_1_grad/mul_1Y^optimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_1_grad/tuple/group_deps*
T0*`
_classV
TRloc:@optimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_1_grad/mul_1
s
Boptimizer/gradients_1/optimizer/clip_by_value_2/Minimum_grad/ShapeShapeoptimizer/Exp*
T0*
out_type0
m
Doptimizer/gradients_1/optimizer/clip_by_value_2/Minimum_grad/Shape_1Const*
dtype0*
valueB 
�
Doptimizer/gradients_1/optimizer/clip_by_value_2/Minimum_grad/Shape_2ShapeMoptimizer/gradients_1/optimizer/clip_by_value_2_grad/tuple/control_dependency*
T0*
out_type0
u
Hoptimizer/gradients_1/optimizer/clip_by_value_2/Minimum_grad/zeros/ConstConst*
dtype0*
valueB
 *    
�
Boptimizer/gradients_1/optimizer/clip_by_value_2/Minimum_grad/zerosFillDoptimizer/gradients_1/optimizer/clip_by_value_2/Minimum_grad/Shape_2Hoptimizer/gradients_1/optimizer/clip_by_value_2/Minimum_grad/zeros/Const*
T0*

index_type0
}
Foptimizer/gradients_1/optimizer/clip_by_value_2/Minimum_grad/LessEqual	LessEqualoptimizer/Expoptimizer/add_10*
T0
�
Roptimizer/gradients_1/optimizer/clip_by_value_2/Minimum_grad/BroadcastGradientArgsBroadcastGradientArgsBoptimizer/gradients_1/optimizer/clip_by_value_2/Minimum_grad/ShapeDoptimizer/gradients_1/optimizer/clip_by_value_2/Minimum_grad/Shape_1*
T0
�
Eoptimizer/gradients_1/optimizer/clip_by_value_2/Minimum_grad/SelectV2SelectV2Foptimizer/gradients_1/optimizer/clip_by_value_2/Minimum_grad/LessEqualMoptimizer/gradients_1/optimizer/clip_by_value_2_grad/tuple/control_dependencyBoptimizer/gradients_1/optimizer/clip_by_value_2/Minimum_grad/zeros*
T0
�
@optimizer/gradients_1/optimizer/clip_by_value_2/Minimum_grad/SumSumEoptimizer/gradients_1/optimizer/clip_by_value_2/Minimum_grad/SelectV2Roptimizer/gradients_1/optimizer/clip_by_value_2/Minimum_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
Doptimizer/gradients_1/optimizer/clip_by_value_2/Minimum_grad/ReshapeReshape@optimizer/gradients_1/optimizer/clip_by_value_2/Minimum_grad/SumBoptimizer/gradients_1/optimizer/clip_by_value_2/Minimum_grad/Shape*
T0*
Tshape0
�
Goptimizer/gradients_1/optimizer/clip_by_value_2/Minimum_grad/SelectV2_1SelectV2Foptimizer/gradients_1/optimizer/clip_by_value_2/Minimum_grad/LessEqualBoptimizer/gradients_1/optimizer/clip_by_value_2/Minimum_grad/zerosMoptimizer/gradients_1/optimizer/clip_by_value_2_grad/tuple/control_dependency*
T0
�
Boptimizer/gradients_1/optimizer/clip_by_value_2/Minimum_grad/Sum_1SumGoptimizer/gradients_1/optimizer/clip_by_value_2/Minimum_grad/SelectV2_1Toptimizer/gradients_1/optimizer/clip_by_value_2/Minimum_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
Foptimizer/gradients_1/optimizer/clip_by_value_2/Minimum_grad/Reshape_1ReshapeBoptimizer/gradients_1/optimizer/clip_by_value_2/Minimum_grad/Sum_1Doptimizer/gradients_1/optimizer/clip_by_value_2/Minimum_grad/Shape_1*
T0*
Tshape0
�
Moptimizer/gradients_1/optimizer/clip_by_value_2/Minimum_grad/tuple/group_depsNoOpE^optimizer/gradients_1/optimizer/clip_by_value_2/Minimum_grad/ReshapeG^optimizer/gradients_1/optimizer/clip_by_value_2/Minimum_grad/Reshape_1
�
Uoptimizer/gradients_1/optimizer/clip_by_value_2/Minimum_grad/tuple/control_dependencyIdentityDoptimizer/gradients_1/optimizer/clip_by_value_2/Minimum_grad/ReshapeN^optimizer/gradients_1/optimizer/clip_by_value_2/Minimum_grad/tuple/group_deps*
T0*W
_classM
KIloc:@optimizer/gradients_1/optimizer/clip_by_value_2/Minimum_grad/Reshape
�
Woptimizer/gradients_1/optimizer/clip_by_value_2/Minimum_grad/tuple/control_dependency_1IdentityFoptimizer/gradients_1/optimizer/clip_by_value_2/Minimum_grad/Reshape_1N^optimizer/gradients_1/optimizer/clip_by_value_2/Minimum_grad/tuple/group_deps*
T0*Y
_classO
MKloc:@optimizer/gradients_1/optimizer/clip_by_value_2/Minimum_grad/Reshape_1
�
?optimizer/gradients_1/optimizer/SquaredDifference_1_grad/scalarConstF^optimizer/gradients_1/optimizer/Maximum_grad/tuple/control_dependency*
dtype0*
valueB
 *   @
�
<optimizer/gradients_1/optimizer/SquaredDifference_1_grad/MulMul?optimizer/gradients_1/optimizer/SquaredDifference_1_grad/scalarEoptimizer/gradients_1/optimizer/Maximum_grad/tuple/control_dependency*
T0
�
<optimizer/gradients_1/optimizer/SquaredDifference_1_grad/subSuboptimizer/extrinsic_returnsoptimizer/Sum_6F^optimizer/gradients_1/optimizer/Maximum_grad/tuple/control_dependency*
T0
�
>optimizer/gradients_1/optimizer/SquaredDifference_1_grad/mul_1Mul<optimizer/gradients_1/optimizer/SquaredDifference_1_grad/Mul<optimizer/gradients_1/optimizer/SquaredDifference_1_grad/sub*
T0
}
>optimizer/gradients_1/optimizer/SquaredDifference_1_grad/ShapeShapeoptimizer/extrinsic_returns*
T0*
out_type0
s
@optimizer/gradients_1/optimizer/SquaredDifference_1_grad/Shape_1Shapeoptimizer/Sum_6*
T0*
out_type0
�
Noptimizer/gradients_1/optimizer/SquaredDifference_1_grad/BroadcastGradientArgsBroadcastGradientArgs>optimizer/gradients_1/optimizer/SquaredDifference_1_grad/Shape@optimizer/gradients_1/optimizer/SquaredDifference_1_grad/Shape_1*
T0
�
<optimizer/gradients_1/optimizer/SquaredDifference_1_grad/SumSum>optimizer/gradients_1/optimizer/SquaredDifference_1_grad/mul_1Noptimizer/gradients_1/optimizer/SquaredDifference_1_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
@optimizer/gradients_1/optimizer/SquaredDifference_1_grad/ReshapeReshape<optimizer/gradients_1/optimizer/SquaredDifference_1_grad/Sum>optimizer/gradients_1/optimizer/SquaredDifference_1_grad/Shape*
T0*
Tshape0
�
>optimizer/gradients_1/optimizer/SquaredDifference_1_grad/Sum_1Sum>optimizer/gradients_1/optimizer/SquaredDifference_1_grad/mul_1Poptimizer/gradients_1/optimizer/SquaredDifference_1_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
Boptimizer/gradients_1/optimizer/SquaredDifference_1_grad/Reshape_1Reshape>optimizer/gradients_1/optimizer/SquaredDifference_1_grad/Sum_1@optimizer/gradients_1/optimizer/SquaredDifference_1_grad/Shape_1*
T0*
Tshape0
�
<optimizer/gradients_1/optimizer/SquaredDifference_1_grad/NegNegBoptimizer/gradients_1/optimizer/SquaredDifference_1_grad/Reshape_1*
T0
�
Ioptimizer/gradients_1/optimizer/SquaredDifference_1_grad/tuple/group_depsNoOp=^optimizer/gradients_1/optimizer/SquaredDifference_1_grad/NegA^optimizer/gradients_1/optimizer/SquaredDifference_1_grad/Reshape
�
Qoptimizer/gradients_1/optimizer/SquaredDifference_1_grad/tuple/control_dependencyIdentity@optimizer/gradients_1/optimizer/SquaredDifference_1_grad/ReshapeJ^optimizer/gradients_1/optimizer/SquaredDifference_1_grad/tuple/group_deps*
T0*S
_classI
GEloc:@optimizer/gradients_1/optimizer/SquaredDifference_1_grad/Reshape
�
Soptimizer/gradients_1/optimizer/SquaredDifference_1_grad/tuple/control_dependency_1Identity<optimizer/gradients_1/optimizer/SquaredDifference_1_grad/NegJ^optimizer/gradients_1/optimizer/SquaredDifference_1_grad/tuple/group_deps*
T0*O
_classE
CAloc:@optimizer/gradients_1/optimizer/SquaredDifference_1_grad/Neg
�
?optimizer/gradients_1/optimizer/SquaredDifference_2_grad/scalarConstH^optimizer/gradients_1/optimizer/Maximum_grad/tuple/control_dependency_1*
dtype0*
valueB
 *   @
�
<optimizer/gradients_1/optimizer/SquaredDifference_2_grad/MulMul?optimizer/gradients_1/optimizer/SquaredDifference_2_grad/scalarGoptimizer/gradients_1/optimizer/Maximum_grad/tuple/control_dependency_1*
T0
�
<optimizer/gradients_1/optimizer/SquaredDifference_2_grad/subSuboptimizer/extrinsic_returnsoptimizer/add_8H^optimizer/gradients_1/optimizer/Maximum_grad/tuple/control_dependency_1*
T0
�
>optimizer/gradients_1/optimizer/SquaredDifference_2_grad/mul_1Mul<optimizer/gradients_1/optimizer/SquaredDifference_2_grad/Mul<optimizer/gradients_1/optimizer/SquaredDifference_2_grad/sub*
T0
}
>optimizer/gradients_1/optimizer/SquaredDifference_2_grad/ShapeShapeoptimizer/extrinsic_returns*
T0*
out_type0
s
@optimizer/gradients_1/optimizer/SquaredDifference_2_grad/Shape_1Shapeoptimizer/add_8*
T0*
out_type0
�
Noptimizer/gradients_1/optimizer/SquaredDifference_2_grad/BroadcastGradientArgsBroadcastGradientArgs>optimizer/gradients_1/optimizer/SquaredDifference_2_grad/Shape@optimizer/gradients_1/optimizer/SquaredDifference_2_grad/Shape_1*
T0
�
<optimizer/gradients_1/optimizer/SquaredDifference_2_grad/SumSum>optimizer/gradients_1/optimizer/SquaredDifference_2_grad/mul_1Noptimizer/gradients_1/optimizer/SquaredDifference_2_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
@optimizer/gradients_1/optimizer/SquaredDifference_2_grad/ReshapeReshape<optimizer/gradients_1/optimizer/SquaredDifference_2_grad/Sum>optimizer/gradients_1/optimizer/SquaredDifference_2_grad/Shape*
T0*
Tshape0
�
>optimizer/gradients_1/optimizer/SquaredDifference_2_grad/Sum_1Sum>optimizer/gradients_1/optimizer/SquaredDifference_2_grad/mul_1Poptimizer/gradients_1/optimizer/SquaredDifference_2_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
Boptimizer/gradients_1/optimizer/SquaredDifference_2_grad/Reshape_1Reshape>optimizer/gradients_1/optimizer/SquaredDifference_2_grad/Sum_1@optimizer/gradients_1/optimizer/SquaredDifference_2_grad/Shape_1*
T0*
Tshape0
�
<optimizer/gradients_1/optimizer/SquaredDifference_2_grad/NegNegBoptimizer/gradients_1/optimizer/SquaredDifference_2_grad/Reshape_1*
T0
�
Ioptimizer/gradients_1/optimizer/SquaredDifference_2_grad/tuple/group_depsNoOp=^optimizer/gradients_1/optimizer/SquaredDifference_2_grad/NegA^optimizer/gradients_1/optimizer/SquaredDifference_2_grad/Reshape
�
Qoptimizer/gradients_1/optimizer/SquaredDifference_2_grad/tuple/control_dependencyIdentity@optimizer/gradients_1/optimizer/SquaredDifference_2_grad/ReshapeJ^optimizer/gradients_1/optimizer/SquaredDifference_2_grad/tuple/group_deps*
T0*S
_classI
GEloc:@optimizer/gradients_1/optimizer/SquaredDifference_2_grad/Reshape
�
Soptimizer/gradients_1/optimizer/SquaredDifference_2_grad/tuple/control_dependency_1Identity<optimizer/gradients_1/optimizer/SquaredDifference_2_grad/NegJ^optimizer/gradients_1/optimizer/SquaredDifference_2_grad/tuple/group_deps*
T0*O
_classE
CAloc:@optimizer/gradients_1/optimizer/SquaredDifference_2_grad/Neg
�
?optimizer/gradients_1/optimizer/SquaredDifference_3_grad/scalarConstH^optimizer/gradients_1/optimizer/Maximum_1_grad/tuple/control_dependency*
dtype0*
valueB
 *   @
�
<optimizer/gradients_1/optimizer/SquaredDifference_3_grad/MulMul?optimizer/gradients_1/optimizer/SquaredDifference_3_grad/scalarGoptimizer/gradients_1/optimizer/Maximum_1_grad/tuple/control_dependency*
T0
�
<optimizer/gradients_1/optimizer/SquaredDifference_3_grad/subSuboptimizer/curiosity_returnsoptimizer/Sum_8H^optimizer/gradients_1/optimizer/Maximum_1_grad/tuple/control_dependency*
T0
�
>optimizer/gradients_1/optimizer/SquaredDifference_3_grad/mul_1Mul<optimizer/gradients_1/optimizer/SquaredDifference_3_grad/Mul<optimizer/gradients_1/optimizer/SquaredDifference_3_grad/sub*
T0
}
>optimizer/gradients_1/optimizer/SquaredDifference_3_grad/ShapeShapeoptimizer/curiosity_returns*
T0*
out_type0
s
@optimizer/gradients_1/optimizer/SquaredDifference_3_grad/Shape_1Shapeoptimizer/Sum_8*
T0*
out_type0
�
Noptimizer/gradients_1/optimizer/SquaredDifference_3_grad/BroadcastGradientArgsBroadcastGradientArgs>optimizer/gradients_1/optimizer/SquaredDifference_3_grad/Shape@optimizer/gradients_1/optimizer/SquaredDifference_3_grad/Shape_1*
T0
�
<optimizer/gradients_1/optimizer/SquaredDifference_3_grad/SumSum>optimizer/gradients_1/optimizer/SquaredDifference_3_grad/mul_1Noptimizer/gradients_1/optimizer/SquaredDifference_3_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
@optimizer/gradients_1/optimizer/SquaredDifference_3_grad/ReshapeReshape<optimizer/gradients_1/optimizer/SquaredDifference_3_grad/Sum>optimizer/gradients_1/optimizer/SquaredDifference_3_grad/Shape*
T0*
Tshape0
�
>optimizer/gradients_1/optimizer/SquaredDifference_3_grad/Sum_1Sum>optimizer/gradients_1/optimizer/SquaredDifference_3_grad/mul_1Poptimizer/gradients_1/optimizer/SquaredDifference_3_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
Boptimizer/gradients_1/optimizer/SquaredDifference_3_grad/Reshape_1Reshape>optimizer/gradients_1/optimizer/SquaredDifference_3_grad/Sum_1@optimizer/gradients_1/optimizer/SquaredDifference_3_grad/Shape_1*
T0*
Tshape0
�
<optimizer/gradients_1/optimizer/SquaredDifference_3_grad/NegNegBoptimizer/gradients_1/optimizer/SquaredDifference_3_grad/Reshape_1*
T0
�
Ioptimizer/gradients_1/optimizer/SquaredDifference_3_grad/tuple/group_depsNoOp=^optimizer/gradients_1/optimizer/SquaredDifference_3_grad/NegA^optimizer/gradients_1/optimizer/SquaredDifference_3_grad/Reshape
�
Qoptimizer/gradients_1/optimizer/SquaredDifference_3_grad/tuple/control_dependencyIdentity@optimizer/gradients_1/optimizer/SquaredDifference_3_grad/ReshapeJ^optimizer/gradients_1/optimizer/SquaredDifference_3_grad/tuple/group_deps*
T0*S
_classI
GEloc:@optimizer/gradients_1/optimizer/SquaredDifference_3_grad/Reshape
�
Soptimizer/gradients_1/optimizer/SquaredDifference_3_grad/tuple/control_dependency_1Identity<optimizer/gradients_1/optimizer/SquaredDifference_3_grad/NegJ^optimizer/gradients_1/optimizer/SquaredDifference_3_grad/tuple/group_deps*
T0*O
_classE
CAloc:@optimizer/gradients_1/optimizer/SquaredDifference_3_grad/Neg
�
?optimizer/gradients_1/optimizer/SquaredDifference_4_grad/scalarConstJ^optimizer/gradients_1/optimizer/Maximum_1_grad/tuple/control_dependency_1*
dtype0*
valueB
 *   @
�
<optimizer/gradients_1/optimizer/SquaredDifference_4_grad/MulMul?optimizer/gradients_1/optimizer/SquaredDifference_4_grad/scalarIoptimizer/gradients_1/optimizer/Maximum_1_grad/tuple/control_dependency_1*
T0
�
<optimizer/gradients_1/optimizer/SquaredDifference_4_grad/subSuboptimizer/curiosity_returnsoptimizer/add_9J^optimizer/gradients_1/optimizer/Maximum_1_grad/tuple/control_dependency_1*
T0
�
>optimizer/gradients_1/optimizer/SquaredDifference_4_grad/mul_1Mul<optimizer/gradients_1/optimizer/SquaredDifference_4_grad/Mul<optimizer/gradients_1/optimizer/SquaredDifference_4_grad/sub*
T0
}
>optimizer/gradients_1/optimizer/SquaredDifference_4_grad/ShapeShapeoptimizer/curiosity_returns*
T0*
out_type0
s
@optimizer/gradients_1/optimizer/SquaredDifference_4_grad/Shape_1Shapeoptimizer/add_9*
T0*
out_type0
�
Noptimizer/gradients_1/optimizer/SquaredDifference_4_grad/BroadcastGradientArgsBroadcastGradientArgs>optimizer/gradients_1/optimizer/SquaredDifference_4_grad/Shape@optimizer/gradients_1/optimizer/SquaredDifference_4_grad/Shape_1*
T0
�
<optimizer/gradients_1/optimizer/SquaredDifference_4_grad/SumSum>optimizer/gradients_1/optimizer/SquaredDifference_4_grad/mul_1Noptimizer/gradients_1/optimizer/SquaredDifference_4_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
@optimizer/gradients_1/optimizer/SquaredDifference_4_grad/ReshapeReshape<optimizer/gradients_1/optimizer/SquaredDifference_4_grad/Sum>optimizer/gradients_1/optimizer/SquaredDifference_4_grad/Shape*
T0*
Tshape0
�
>optimizer/gradients_1/optimizer/SquaredDifference_4_grad/Sum_1Sum>optimizer/gradients_1/optimizer/SquaredDifference_4_grad/mul_1Poptimizer/gradients_1/optimizer/SquaredDifference_4_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
Boptimizer/gradients_1/optimizer/SquaredDifference_4_grad/Reshape_1Reshape>optimizer/gradients_1/optimizer/SquaredDifference_4_grad/Sum_1@optimizer/gradients_1/optimizer/SquaredDifference_4_grad/Shape_1*
T0*
Tshape0
�
<optimizer/gradients_1/optimizer/SquaredDifference_4_grad/NegNegBoptimizer/gradients_1/optimizer/SquaredDifference_4_grad/Reshape_1*
T0
�
Ioptimizer/gradients_1/optimizer/SquaredDifference_4_grad/tuple/group_depsNoOp=^optimizer/gradients_1/optimizer/SquaredDifference_4_grad/NegA^optimizer/gradients_1/optimizer/SquaredDifference_4_grad/Reshape
�
Qoptimizer/gradients_1/optimizer/SquaredDifference_4_grad/tuple/control_dependencyIdentity@optimizer/gradients_1/optimizer/SquaredDifference_4_grad/ReshapeJ^optimizer/gradients_1/optimizer/SquaredDifference_4_grad/tuple/group_deps*
T0*S
_classI
GEloc:@optimizer/gradients_1/optimizer/SquaredDifference_4_grad/Reshape
�
Soptimizer/gradients_1/optimizer/SquaredDifference_4_grad/tuple/control_dependency_1Identity<optimizer/gradients_1/optimizer/SquaredDifference_4_grad/NegJ^optimizer/gradients_1/optimizer/SquaredDifference_4_grad/tuple/group_deps*
T0*O
_classE
CAloc:@optimizer/gradients_1/optimizer/SquaredDifference_4_grad/Neg
�
Soptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits/Reshape_grad/ShapeShapepolicy_1/strided_slice_5*
T0*
out_type0
�
Uoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits/Reshape_grad/ReshapeReshape^optimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_grad/tuple/control_dependencySoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits/Reshape_grad/Shape*
T0*
Tshape0
�
Uoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits/Reshape_1_grad/ShapeShapepolicy_1/Softmax_2*
T0*
out_type0
�
Woptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits/Reshape_1_grad/ReshapeReshape`optimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_grad/tuple/control_dependency_1Uoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits/Reshape_1_grad/Shape*
T0*
Tshape0
�
Uoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_1/Reshape_grad/ShapeShapepolicy_1/strided_slice_7*
T0*
out_type0
�
Woptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_1/Reshape_grad/ReshapeReshape`optimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_1_grad/tuple/control_dependencyUoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_1/Reshape_grad/Shape*
T0*
Tshape0
�
Woptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_1/Reshape_1_grad/ShapeShapepolicy_1/Softmax_3*
T0*
out_type0
�
Yoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_1/Reshape_1_grad/ReshapeReshapeboptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_1_grad/tuple/control_dependency_1Woptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_1/Reshape_1_grad/Shape*
T0*
Tshape0
�
optimizer/gradients_1/AddNAddNCoptimizer/gradients_1/optimizer/mul_7_grad/tuple/control_dependencyUoptimizer/gradients_1/optimizer/clip_by_value_2/Minimum_grad/tuple/control_dependency*
N*
T0*E
_class;
97loc:@optimizer/gradients_1/optimizer/mul_7_grad/Reshape
g
,optimizer/gradients_1/optimizer/Exp_grad/mulMuloptimizer/gradients_1/AddNoptimizer/Exp*
T0
u
0optimizer/gradients_1/optimizer/Sum_6_grad/ShapeShape!optimizer/extrinsic_value/BiasAdd*
T0*
out_type0
�
/optimizer/gradients_1/optimizer/Sum_6_grad/SizeConst*C
_class9
75loc:@optimizer/gradients_1/optimizer/Sum_6_grad/Shape*
dtype0*
value	B :
�
.optimizer/gradients_1/optimizer/Sum_6_grad/addAddV2!optimizer/Sum_6/reduction_indices/optimizer/gradients_1/optimizer/Sum_6_grad/Size*
T0*C
_class9
75loc:@optimizer/gradients_1/optimizer/Sum_6_grad/Shape
�
.optimizer/gradients_1/optimizer/Sum_6_grad/modFloorMod.optimizer/gradients_1/optimizer/Sum_6_grad/add/optimizer/gradients_1/optimizer/Sum_6_grad/Size*
T0*C
_class9
75loc:@optimizer/gradients_1/optimizer/Sum_6_grad/Shape
�
2optimizer/gradients_1/optimizer/Sum_6_grad/Shape_1Const*C
_class9
75loc:@optimizer/gradients_1/optimizer/Sum_6_grad/Shape*
dtype0*
valueB 
�
6optimizer/gradients_1/optimizer/Sum_6_grad/range/startConst*C
_class9
75loc:@optimizer/gradients_1/optimizer/Sum_6_grad/Shape*
dtype0*
value	B : 
�
6optimizer/gradients_1/optimizer/Sum_6_grad/range/deltaConst*C
_class9
75loc:@optimizer/gradients_1/optimizer/Sum_6_grad/Shape*
dtype0*
value	B :
�
0optimizer/gradients_1/optimizer/Sum_6_grad/rangeRange6optimizer/gradients_1/optimizer/Sum_6_grad/range/start/optimizer/gradients_1/optimizer/Sum_6_grad/Size6optimizer/gradients_1/optimizer/Sum_6_grad/range/delta*

Tidx0*C
_class9
75loc:@optimizer/gradients_1/optimizer/Sum_6_grad/Shape
�
5optimizer/gradients_1/optimizer/Sum_6_grad/Fill/valueConst*C
_class9
75loc:@optimizer/gradients_1/optimizer/Sum_6_grad/Shape*
dtype0*
value	B :
�
/optimizer/gradients_1/optimizer/Sum_6_grad/FillFill2optimizer/gradients_1/optimizer/Sum_6_grad/Shape_15optimizer/gradients_1/optimizer/Sum_6_grad/Fill/value*
T0*C
_class9
75loc:@optimizer/gradients_1/optimizer/Sum_6_grad/Shape*

index_type0
�
8optimizer/gradients_1/optimizer/Sum_6_grad/DynamicStitchDynamicStitch0optimizer/gradients_1/optimizer/Sum_6_grad/range.optimizer/gradients_1/optimizer/Sum_6_grad/mod0optimizer/gradients_1/optimizer/Sum_6_grad/Shape/optimizer/gradients_1/optimizer/Sum_6_grad/Fill*
N*
T0*C
_class9
75loc:@optimizer/gradients_1/optimizer/Sum_6_grad/Shape
�
2optimizer/gradients_1/optimizer/Sum_6_grad/ReshapeReshapeSoptimizer/gradients_1/optimizer/SquaredDifference_1_grad/tuple/control_dependency_18optimizer/gradients_1/optimizer/Sum_6_grad/DynamicStitch*
T0*
Tshape0
�
6optimizer/gradients_1/optimizer/Sum_6_grad/BroadcastToBroadcastTo2optimizer/gradients_1/optimizer/Sum_6_grad/Reshape0optimizer/gradients_1/optimizer/Sum_6_grad/Shape*
T0*

Tidx0
v
0optimizer/gradients_1/optimizer/add_8_grad/ShapeShape"optimizer/extrinsic_value_estimate*
T0*
out_type0
m
2optimizer/gradients_1/optimizer/add_8_grad/Shape_1Shapeoptimizer/clip_by_value*
T0*
out_type0
�
@optimizer/gradients_1/optimizer/add_8_grad/BroadcastGradientArgsBroadcastGradientArgs0optimizer/gradients_1/optimizer/add_8_grad/Shape2optimizer/gradients_1/optimizer/add_8_grad/Shape_1*
T0
�
.optimizer/gradients_1/optimizer/add_8_grad/SumSumSoptimizer/gradients_1/optimizer/SquaredDifference_2_grad/tuple/control_dependency_1@optimizer/gradients_1/optimizer/add_8_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
2optimizer/gradients_1/optimizer/add_8_grad/ReshapeReshape.optimizer/gradients_1/optimizer/add_8_grad/Sum0optimizer/gradients_1/optimizer/add_8_grad/Shape*
T0*
Tshape0
�
0optimizer/gradients_1/optimizer/add_8_grad/Sum_1SumSoptimizer/gradients_1/optimizer/SquaredDifference_2_grad/tuple/control_dependency_1Boptimizer/gradients_1/optimizer/add_8_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
4optimizer/gradients_1/optimizer/add_8_grad/Reshape_1Reshape0optimizer/gradients_1/optimizer/add_8_grad/Sum_12optimizer/gradients_1/optimizer/add_8_grad/Shape_1*
T0*
Tshape0
�
;optimizer/gradients_1/optimizer/add_8_grad/tuple/group_depsNoOp3^optimizer/gradients_1/optimizer/add_8_grad/Reshape5^optimizer/gradients_1/optimizer/add_8_grad/Reshape_1
�
Coptimizer/gradients_1/optimizer/add_8_grad/tuple/control_dependencyIdentity2optimizer/gradients_1/optimizer/add_8_grad/Reshape<^optimizer/gradients_1/optimizer/add_8_grad/tuple/group_deps*
T0*E
_class;
97loc:@optimizer/gradients_1/optimizer/add_8_grad/Reshape
�
Eoptimizer/gradients_1/optimizer/add_8_grad/tuple/control_dependency_1Identity4optimizer/gradients_1/optimizer/add_8_grad/Reshape_1<^optimizer/gradients_1/optimizer/add_8_grad/tuple/group_deps*
T0*G
_class=
;9loc:@optimizer/gradients_1/optimizer/add_8_grad/Reshape_1
u
0optimizer/gradients_1/optimizer/Sum_8_grad/ShapeShape!optimizer/curiosity_value/BiasAdd*
T0*
out_type0
�
/optimizer/gradients_1/optimizer/Sum_8_grad/SizeConst*C
_class9
75loc:@optimizer/gradients_1/optimizer/Sum_8_grad/Shape*
dtype0*
value	B :
�
.optimizer/gradients_1/optimizer/Sum_8_grad/addAddV2!optimizer/Sum_8/reduction_indices/optimizer/gradients_1/optimizer/Sum_8_grad/Size*
T0*C
_class9
75loc:@optimizer/gradients_1/optimizer/Sum_8_grad/Shape
�
.optimizer/gradients_1/optimizer/Sum_8_grad/modFloorMod.optimizer/gradients_1/optimizer/Sum_8_grad/add/optimizer/gradients_1/optimizer/Sum_8_grad/Size*
T0*C
_class9
75loc:@optimizer/gradients_1/optimizer/Sum_8_grad/Shape
�
2optimizer/gradients_1/optimizer/Sum_8_grad/Shape_1Const*C
_class9
75loc:@optimizer/gradients_1/optimizer/Sum_8_grad/Shape*
dtype0*
valueB 
�
6optimizer/gradients_1/optimizer/Sum_8_grad/range/startConst*C
_class9
75loc:@optimizer/gradients_1/optimizer/Sum_8_grad/Shape*
dtype0*
value	B : 
�
6optimizer/gradients_1/optimizer/Sum_8_grad/range/deltaConst*C
_class9
75loc:@optimizer/gradients_1/optimizer/Sum_8_grad/Shape*
dtype0*
value	B :
�
0optimizer/gradients_1/optimizer/Sum_8_grad/rangeRange6optimizer/gradients_1/optimizer/Sum_8_grad/range/start/optimizer/gradients_1/optimizer/Sum_8_grad/Size6optimizer/gradients_1/optimizer/Sum_8_grad/range/delta*

Tidx0*C
_class9
75loc:@optimizer/gradients_1/optimizer/Sum_8_grad/Shape
�
5optimizer/gradients_1/optimizer/Sum_8_grad/Fill/valueConst*C
_class9
75loc:@optimizer/gradients_1/optimizer/Sum_8_grad/Shape*
dtype0*
value	B :
�
/optimizer/gradients_1/optimizer/Sum_8_grad/FillFill2optimizer/gradients_1/optimizer/Sum_8_grad/Shape_15optimizer/gradients_1/optimizer/Sum_8_grad/Fill/value*
T0*C
_class9
75loc:@optimizer/gradients_1/optimizer/Sum_8_grad/Shape*

index_type0
�
8optimizer/gradients_1/optimizer/Sum_8_grad/DynamicStitchDynamicStitch0optimizer/gradients_1/optimizer/Sum_8_grad/range.optimizer/gradients_1/optimizer/Sum_8_grad/mod0optimizer/gradients_1/optimizer/Sum_8_grad/Shape/optimizer/gradients_1/optimizer/Sum_8_grad/Fill*
N*
T0*C
_class9
75loc:@optimizer/gradients_1/optimizer/Sum_8_grad/Shape
�
2optimizer/gradients_1/optimizer/Sum_8_grad/ReshapeReshapeSoptimizer/gradients_1/optimizer/SquaredDifference_3_grad/tuple/control_dependency_18optimizer/gradients_1/optimizer/Sum_8_grad/DynamicStitch*
T0*
Tshape0
�
6optimizer/gradients_1/optimizer/Sum_8_grad/BroadcastToBroadcastTo2optimizer/gradients_1/optimizer/Sum_8_grad/Reshape0optimizer/gradients_1/optimizer/Sum_8_grad/Shape*
T0*

Tidx0
v
0optimizer/gradients_1/optimizer/add_9_grad/ShapeShape"optimizer/curiosity_value_estimate*
T0*
out_type0
o
2optimizer/gradients_1/optimizer/add_9_grad/Shape_1Shapeoptimizer/clip_by_value_1*
T0*
out_type0
�
@optimizer/gradients_1/optimizer/add_9_grad/BroadcastGradientArgsBroadcastGradientArgs0optimizer/gradients_1/optimizer/add_9_grad/Shape2optimizer/gradients_1/optimizer/add_9_grad/Shape_1*
T0
�
.optimizer/gradients_1/optimizer/add_9_grad/SumSumSoptimizer/gradients_1/optimizer/SquaredDifference_4_grad/tuple/control_dependency_1@optimizer/gradients_1/optimizer/add_9_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
2optimizer/gradients_1/optimizer/add_9_grad/ReshapeReshape.optimizer/gradients_1/optimizer/add_9_grad/Sum0optimizer/gradients_1/optimizer/add_9_grad/Shape*
T0*
Tshape0
�
0optimizer/gradients_1/optimizer/add_9_grad/Sum_1SumSoptimizer/gradients_1/optimizer/SquaredDifference_4_grad/tuple/control_dependency_1Boptimizer/gradients_1/optimizer/add_9_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
4optimizer/gradients_1/optimizer/add_9_grad/Reshape_1Reshape0optimizer/gradients_1/optimizer/add_9_grad/Sum_12optimizer/gradients_1/optimizer/add_9_grad/Shape_1*
T0*
Tshape0
�
;optimizer/gradients_1/optimizer/add_9_grad/tuple/group_depsNoOp3^optimizer/gradients_1/optimizer/add_9_grad/Reshape5^optimizer/gradients_1/optimizer/add_9_grad/Reshape_1
�
Coptimizer/gradients_1/optimizer/add_9_grad/tuple/control_dependencyIdentity2optimizer/gradients_1/optimizer/add_9_grad/Reshape<^optimizer/gradients_1/optimizer/add_9_grad/tuple/group_deps*
T0*E
_class;
97loc:@optimizer/gradients_1/optimizer/add_9_grad/Reshape
�
Eoptimizer/gradients_1/optimizer/add_9_grad/tuple/control_dependency_1Identity4optimizer/gradients_1/optimizer/add_9_grad/Reshape_1<^optimizer/gradients_1/optimizer/add_9_grad/tuple/group_deps*
T0*G
_class=
;9loc:@optimizer/gradients_1/optimizer/add_9_grad/Reshape_1
n
9optimizer/gradients_1/policy_1/strided_slice_5_grad/ShapeShapepolicy_1/concat_2*
T0*
out_type0

Joptimizer/gradients_1/policy_1/strided_slice_5_grad/StridedSliceGrad/beginConst*
dtype0*
valueB"        
}
Hoptimizer/gradients_1/policy_1/strided_slice_5_grad/StridedSliceGrad/endConst*
dtype0*
valueB"       
�
Loptimizer/gradients_1/policy_1/strided_slice_5_grad/StridedSliceGrad/stridesConst*
dtype0*
valueB"      
�
Doptimizer/gradients_1/policy_1/strided_slice_5_grad/StridedSliceGradStridedSliceGrad9optimizer/gradients_1/policy_1/strided_slice_5_grad/ShapeJoptimizer/gradients_1/policy_1/strided_slice_5_grad/StridedSliceGrad/beginHoptimizer/gradients_1/policy_1/strided_slice_5_grad/StridedSliceGrad/endLoptimizer/gradients_1/policy_1/strided_slice_5_grad/StridedSliceGrad/stridesUoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits/Reshape_grad/Reshape*
Index0*
T0*

begin_mask*
ellipsis_mask *
end_mask*
new_axis_mask *
shrink_axis_mask 
�
1optimizer/gradients_1/policy_1/Softmax_2_grad/mulMulWoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits/Reshape_1_grad/Reshapepolicy_1/Softmax_2*
T0
v
Coptimizer/gradients_1/policy_1/Softmax_2_grad/Sum/reduction_indicesConst*
dtype0*
valueB :
���������
�
1optimizer/gradients_1/policy_1/Softmax_2_grad/SumSum1optimizer/gradients_1/policy_1/Softmax_2_grad/mulCoptimizer/gradients_1/policy_1/Softmax_2_grad/Sum/reduction_indices*
T0*

Tidx0*
	keep_dims(
�
1optimizer/gradients_1/policy_1/Softmax_2_grad/subSubWoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits/Reshape_1_grad/Reshape1optimizer/gradients_1/policy_1/Softmax_2_grad/Sum*
T0
�
3optimizer/gradients_1/policy_1/Softmax_2_grad/mul_1Mul1optimizer/gradients_1/policy_1/Softmax_2_grad/subpolicy_1/Softmax_2*
T0
n
9optimizer/gradients_1/policy_1/strided_slice_7_grad/ShapeShapepolicy_1/concat_2*
T0*
out_type0

Joptimizer/gradients_1/policy_1/strided_slice_7_grad/StridedSliceGrad/beginConst*
dtype0*
valueB"       
}
Hoptimizer/gradients_1/policy_1/strided_slice_7_grad/StridedSliceGrad/endConst*
dtype0*
valueB"       
�
Loptimizer/gradients_1/policy_1/strided_slice_7_grad/StridedSliceGrad/stridesConst*
dtype0*
valueB"      
�
Doptimizer/gradients_1/policy_1/strided_slice_7_grad/StridedSliceGradStridedSliceGrad9optimizer/gradients_1/policy_1/strided_slice_7_grad/ShapeJoptimizer/gradients_1/policy_1/strided_slice_7_grad/StridedSliceGrad/beginHoptimizer/gradients_1/policy_1/strided_slice_7_grad/StridedSliceGrad/endLoptimizer/gradients_1/policy_1/strided_slice_7_grad/StridedSliceGrad/stridesWoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_1/Reshape_grad/Reshape*
Index0*
T0*

begin_mask*
ellipsis_mask *
end_mask*
new_axis_mask *
shrink_axis_mask 
�
1optimizer/gradients_1/policy_1/Softmax_3_grad/mulMulYoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_1/Reshape_1_grad/Reshapepolicy_1/Softmax_3*
T0
v
Coptimizer/gradients_1/policy_1/Softmax_3_grad/Sum/reduction_indicesConst*
dtype0*
valueB :
���������
�
1optimizer/gradients_1/policy_1/Softmax_3_grad/SumSum1optimizer/gradients_1/policy_1/Softmax_3_grad/mulCoptimizer/gradients_1/policy_1/Softmax_3_grad/Sum/reduction_indices*
T0*

Tidx0*
	keep_dims(
�
1optimizer/gradients_1/policy_1/Softmax_3_grad/subSubYoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_1/Reshape_1_grad/Reshape1optimizer/gradients_1/policy_1/Softmax_3_grad/Sum*
T0
�
3optimizer/gradients_1/policy_1/Softmax_3_grad/mul_1Mul1optimizer/gradients_1/policy_1/Softmax_3_grad/subpolicy_1/Softmax_3*
T0
b
0optimizer/gradients_1/optimizer/sub_2_grad/ShapeShapepolicy_1/Sum_3*
T0*
out_type0
e
2optimizer/gradients_1/optimizer/sub_2_grad/Shape_1Shapeoptimizer/Sum_4*
T0*
out_type0
�
@optimizer/gradients_1/optimizer/sub_2_grad/BroadcastGradientArgsBroadcastGradientArgs0optimizer/gradients_1/optimizer/sub_2_grad/Shape2optimizer/gradients_1/optimizer/sub_2_grad/Shape_1*
T0
�
.optimizer/gradients_1/optimizer/sub_2_grad/SumSum,optimizer/gradients_1/optimizer/Exp_grad/mul@optimizer/gradients_1/optimizer/sub_2_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
2optimizer/gradients_1/optimizer/sub_2_grad/ReshapeReshape.optimizer/gradients_1/optimizer/sub_2_grad/Sum0optimizer/gradients_1/optimizer/sub_2_grad/Shape*
T0*
Tshape0
l
.optimizer/gradients_1/optimizer/sub_2_grad/NegNeg,optimizer/gradients_1/optimizer/Exp_grad/mul*
T0
�
0optimizer/gradients_1/optimizer/sub_2_grad/Sum_1Sum.optimizer/gradients_1/optimizer/sub_2_grad/NegBoptimizer/gradients_1/optimizer/sub_2_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
4optimizer/gradients_1/optimizer/sub_2_grad/Reshape_1Reshape0optimizer/gradients_1/optimizer/sub_2_grad/Sum_12optimizer/gradients_1/optimizer/sub_2_grad/Shape_1*
T0*
Tshape0
�
;optimizer/gradients_1/optimizer/sub_2_grad/tuple/group_depsNoOp3^optimizer/gradients_1/optimizer/sub_2_grad/Reshape5^optimizer/gradients_1/optimizer/sub_2_grad/Reshape_1
�
Coptimizer/gradients_1/optimizer/sub_2_grad/tuple/control_dependencyIdentity2optimizer/gradients_1/optimizer/sub_2_grad/Reshape<^optimizer/gradients_1/optimizer/sub_2_grad/tuple/group_deps*
T0*E
_class;
97loc:@optimizer/gradients_1/optimizer/sub_2_grad/Reshape
�
Eoptimizer/gradients_1/optimizer/sub_2_grad/tuple/control_dependency_1Identity4optimizer/gradients_1/optimizer/sub_2_grad/Reshape_1<^optimizer/gradients_1/optimizer/sub_2_grad/tuple/group_deps*
T0*G
_class=
;9loc:@optimizer/gradients_1/optimizer/sub_2_grad/Reshape_1
{
8optimizer/gradients_1/optimizer/clip_by_value_grad/ShapeShapeoptimizer/clip_by_value/Minimum*
T0*
out_type0
c
:optimizer/gradients_1/optimizer/clip_by_value_grad/Shape_1Const*
dtype0*
valueB 
�
:optimizer/gradients_1/optimizer/clip_by_value_grad/Shape_2ShapeEoptimizer/gradients_1/optimizer/add_8_grad/tuple/control_dependency_1*
T0*
out_type0
k
>optimizer/gradients_1/optimizer/clip_by_value_grad/zeros/ConstConst*
dtype0*
valueB
 *    
�
8optimizer/gradients_1/optimizer/clip_by_value_grad/zerosFill:optimizer/gradients_1/optimizer/clip_by_value_grad/Shape_2>optimizer/gradients_1/optimizer/clip_by_value_grad/zeros/Const*
T0*

index_type0
�
?optimizer/gradients_1/optimizer/clip_by_value_grad/GreaterEqualGreaterEqualoptimizer/clip_by_value/Minimumoptimizer/Neg_3*
T0
�
Hoptimizer/gradients_1/optimizer/clip_by_value_grad/BroadcastGradientArgsBroadcastGradientArgs8optimizer/gradients_1/optimizer/clip_by_value_grad/Shape:optimizer/gradients_1/optimizer/clip_by_value_grad/Shape_1*
T0
�
;optimizer/gradients_1/optimizer/clip_by_value_grad/SelectV2SelectV2?optimizer/gradients_1/optimizer/clip_by_value_grad/GreaterEqualEoptimizer/gradients_1/optimizer/add_8_grad/tuple/control_dependency_18optimizer/gradients_1/optimizer/clip_by_value_grad/zeros*
T0
�
6optimizer/gradients_1/optimizer/clip_by_value_grad/SumSum;optimizer/gradients_1/optimizer/clip_by_value_grad/SelectV2Hoptimizer/gradients_1/optimizer/clip_by_value_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
:optimizer/gradients_1/optimizer/clip_by_value_grad/ReshapeReshape6optimizer/gradients_1/optimizer/clip_by_value_grad/Sum8optimizer/gradients_1/optimizer/clip_by_value_grad/Shape*
T0*
Tshape0
�
=optimizer/gradients_1/optimizer/clip_by_value_grad/SelectV2_1SelectV2?optimizer/gradients_1/optimizer/clip_by_value_grad/GreaterEqual8optimizer/gradients_1/optimizer/clip_by_value_grad/zerosEoptimizer/gradients_1/optimizer/add_8_grad/tuple/control_dependency_1*
T0
�
8optimizer/gradients_1/optimizer/clip_by_value_grad/Sum_1Sum=optimizer/gradients_1/optimizer/clip_by_value_grad/SelectV2_1Joptimizer/gradients_1/optimizer/clip_by_value_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
<optimizer/gradients_1/optimizer/clip_by_value_grad/Reshape_1Reshape8optimizer/gradients_1/optimizer/clip_by_value_grad/Sum_1:optimizer/gradients_1/optimizer/clip_by_value_grad/Shape_1*
T0*
Tshape0
�
Coptimizer/gradients_1/optimizer/clip_by_value_grad/tuple/group_depsNoOp;^optimizer/gradients_1/optimizer/clip_by_value_grad/Reshape=^optimizer/gradients_1/optimizer/clip_by_value_grad/Reshape_1
�
Koptimizer/gradients_1/optimizer/clip_by_value_grad/tuple/control_dependencyIdentity:optimizer/gradients_1/optimizer/clip_by_value_grad/ReshapeD^optimizer/gradients_1/optimizer/clip_by_value_grad/tuple/group_deps*
T0*M
_classC
A?loc:@optimizer/gradients_1/optimizer/clip_by_value_grad/Reshape
�
Moptimizer/gradients_1/optimizer/clip_by_value_grad/tuple/control_dependency_1Identity<optimizer/gradients_1/optimizer/clip_by_value_grad/Reshape_1D^optimizer/gradients_1/optimizer/clip_by_value_grad/tuple/group_deps*
T0*O
_classE
CAloc:@optimizer/gradients_1/optimizer/clip_by_value_grad/Reshape_1

:optimizer/gradients_1/optimizer/clip_by_value_1_grad/ShapeShape!optimizer/clip_by_value_1/Minimum*
T0*
out_type0
e
<optimizer/gradients_1/optimizer/clip_by_value_1_grad/Shape_1Const*
dtype0*
valueB 
�
<optimizer/gradients_1/optimizer/clip_by_value_1_grad/Shape_2ShapeEoptimizer/gradients_1/optimizer/add_9_grad/tuple/control_dependency_1*
T0*
out_type0
m
@optimizer/gradients_1/optimizer/clip_by_value_1_grad/zeros/ConstConst*
dtype0*
valueB
 *    
�
:optimizer/gradients_1/optimizer/clip_by_value_1_grad/zerosFill<optimizer/gradients_1/optimizer/clip_by_value_1_grad/Shape_2@optimizer/gradients_1/optimizer/clip_by_value_1_grad/zeros/Const*
T0*

index_type0
�
Aoptimizer/gradients_1/optimizer/clip_by_value_1_grad/GreaterEqualGreaterEqual!optimizer/clip_by_value_1/Minimumoptimizer/Neg_4*
T0
�
Joptimizer/gradients_1/optimizer/clip_by_value_1_grad/BroadcastGradientArgsBroadcastGradientArgs:optimizer/gradients_1/optimizer/clip_by_value_1_grad/Shape<optimizer/gradients_1/optimizer/clip_by_value_1_grad/Shape_1*
T0
�
=optimizer/gradients_1/optimizer/clip_by_value_1_grad/SelectV2SelectV2Aoptimizer/gradients_1/optimizer/clip_by_value_1_grad/GreaterEqualEoptimizer/gradients_1/optimizer/add_9_grad/tuple/control_dependency_1:optimizer/gradients_1/optimizer/clip_by_value_1_grad/zeros*
T0
�
8optimizer/gradients_1/optimizer/clip_by_value_1_grad/SumSum=optimizer/gradients_1/optimizer/clip_by_value_1_grad/SelectV2Joptimizer/gradients_1/optimizer/clip_by_value_1_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
<optimizer/gradients_1/optimizer/clip_by_value_1_grad/ReshapeReshape8optimizer/gradients_1/optimizer/clip_by_value_1_grad/Sum:optimizer/gradients_1/optimizer/clip_by_value_1_grad/Shape*
T0*
Tshape0
�
?optimizer/gradients_1/optimizer/clip_by_value_1_grad/SelectV2_1SelectV2Aoptimizer/gradients_1/optimizer/clip_by_value_1_grad/GreaterEqual:optimizer/gradients_1/optimizer/clip_by_value_1_grad/zerosEoptimizer/gradients_1/optimizer/add_9_grad/tuple/control_dependency_1*
T0
�
:optimizer/gradients_1/optimizer/clip_by_value_1_grad/Sum_1Sum?optimizer/gradients_1/optimizer/clip_by_value_1_grad/SelectV2_1Loptimizer/gradients_1/optimizer/clip_by_value_1_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
>optimizer/gradients_1/optimizer/clip_by_value_1_grad/Reshape_1Reshape:optimizer/gradients_1/optimizer/clip_by_value_1_grad/Sum_1<optimizer/gradients_1/optimizer/clip_by_value_1_grad/Shape_1*
T0*
Tshape0
�
Eoptimizer/gradients_1/optimizer/clip_by_value_1_grad/tuple/group_depsNoOp=^optimizer/gradients_1/optimizer/clip_by_value_1_grad/Reshape?^optimizer/gradients_1/optimizer/clip_by_value_1_grad/Reshape_1
�
Moptimizer/gradients_1/optimizer/clip_by_value_1_grad/tuple/control_dependencyIdentity<optimizer/gradients_1/optimizer/clip_by_value_1_grad/ReshapeF^optimizer/gradients_1/optimizer/clip_by_value_1_grad/tuple/group_deps*
T0*O
_classE
CAloc:@optimizer/gradients_1/optimizer/clip_by_value_1_grad/Reshape
�
Ooptimizer/gradients_1/optimizer/clip_by_value_1_grad/tuple/control_dependency_1Identity>optimizer/gradients_1/optimizer/clip_by_value_1_grad/Reshape_1F^optimizer/gradients_1/optimizer/clip_by_value_1_grad/tuple/group_deps*
T0*Q
_classG
ECloc:@optimizer/gradients_1/optimizer/clip_by_value_1_grad/Reshape_1
n
9optimizer/gradients_1/policy_1/strided_slice_4_grad/ShapeShapepolicy_1/concat_2*
T0*
out_type0

Joptimizer/gradients_1/policy_1/strided_slice_4_grad/StridedSliceGrad/beginConst*
dtype0*
valueB"        
}
Hoptimizer/gradients_1/policy_1/strided_slice_4_grad/StridedSliceGrad/endConst*
dtype0*
valueB"       
�
Loptimizer/gradients_1/policy_1/strided_slice_4_grad/StridedSliceGrad/stridesConst*
dtype0*
valueB"      
�
Doptimizer/gradients_1/policy_1/strided_slice_4_grad/StridedSliceGradStridedSliceGrad9optimizer/gradients_1/policy_1/strided_slice_4_grad/ShapeJoptimizer/gradients_1/policy_1/strided_slice_4_grad/StridedSliceGrad/beginHoptimizer/gradients_1/policy_1/strided_slice_4_grad/StridedSliceGrad/endLoptimizer/gradients_1/policy_1/strided_slice_4_grad/StridedSliceGrad/strides3optimizer/gradients_1/policy_1/Softmax_2_grad/mul_1*
Index0*
T0*

begin_mask*
ellipsis_mask *
end_mask*
new_axis_mask *
shrink_axis_mask 
n
9optimizer/gradients_1/policy_1/strided_slice_6_grad/ShapeShapepolicy_1/concat_2*
T0*
out_type0

Joptimizer/gradients_1/policy_1/strided_slice_6_grad/StridedSliceGrad/beginConst*
dtype0*
valueB"       
}
Hoptimizer/gradients_1/policy_1/strided_slice_6_grad/StridedSliceGrad/endConst*
dtype0*
valueB"       
�
Loptimizer/gradients_1/policy_1/strided_slice_6_grad/StridedSliceGrad/stridesConst*
dtype0*
valueB"      
�
Doptimizer/gradients_1/policy_1/strided_slice_6_grad/StridedSliceGradStridedSliceGrad9optimizer/gradients_1/policy_1/strided_slice_6_grad/ShapeJoptimizer/gradients_1/policy_1/strided_slice_6_grad/StridedSliceGrad/beginHoptimizer/gradients_1/policy_1/strided_slice_6_grad/StridedSliceGrad/endLoptimizer/gradients_1/policy_1/strided_slice_6_grad/StridedSliceGrad/strides3optimizer/gradients_1/policy_1/Softmax_3_grad/mul_1*
Index0*
T0*

begin_mask*
ellipsis_mask *
end_mask*
new_axis_mask *
shrink_axis_mask 
c
/optimizer/gradients_1/policy_1/Sum_3_grad/ShapeShapepolicy_1/stack_1*
T0*
out_type0
�
5optimizer/gradients_1/policy_1/Sum_3_grad/BroadcastToBroadcastToCoptimizer/gradients_1/optimizer/sub_2_grad/tuple/control_dependency/optimizer/gradients_1/policy_1/Sum_3_grad/Shape*
T0*

Tidx0
q
@optimizer/gradients_1/optimizer/clip_by_value/Minimum_grad/ShapeShapeoptimizer/sub*
T0*
out_type0
k
Boptimizer/gradients_1/optimizer/clip_by_value/Minimum_grad/Shape_1Const*
dtype0*
valueB 
�
Boptimizer/gradients_1/optimizer/clip_by_value/Minimum_grad/Shape_2ShapeKoptimizer/gradients_1/optimizer/clip_by_value_grad/tuple/control_dependency*
T0*
out_type0
s
Foptimizer/gradients_1/optimizer/clip_by_value/Minimum_grad/zeros/ConstConst*
dtype0*
valueB
 *    
�
@optimizer/gradients_1/optimizer/clip_by_value/Minimum_grad/zerosFillBoptimizer/gradients_1/optimizer/clip_by_value/Minimum_grad/Shape_2Foptimizer/gradients_1/optimizer/clip_by_value/Minimum_grad/zeros/Const*
T0*

index_type0
�
Doptimizer/gradients_1/optimizer/clip_by_value/Minimum_grad/LessEqual	LessEqualoptimizer/suboptimizer/Variable_1/read*
T0
�
Poptimizer/gradients_1/optimizer/clip_by_value/Minimum_grad/BroadcastGradientArgsBroadcastGradientArgs@optimizer/gradients_1/optimizer/clip_by_value/Minimum_grad/ShapeBoptimizer/gradients_1/optimizer/clip_by_value/Minimum_grad/Shape_1*
T0
�
Coptimizer/gradients_1/optimizer/clip_by_value/Minimum_grad/SelectV2SelectV2Doptimizer/gradients_1/optimizer/clip_by_value/Minimum_grad/LessEqualKoptimizer/gradients_1/optimizer/clip_by_value_grad/tuple/control_dependency@optimizer/gradients_1/optimizer/clip_by_value/Minimum_grad/zeros*
T0
�
>optimizer/gradients_1/optimizer/clip_by_value/Minimum_grad/SumSumCoptimizer/gradients_1/optimizer/clip_by_value/Minimum_grad/SelectV2Poptimizer/gradients_1/optimizer/clip_by_value/Minimum_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
Boptimizer/gradients_1/optimizer/clip_by_value/Minimum_grad/ReshapeReshape>optimizer/gradients_1/optimizer/clip_by_value/Minimum_grad/Sum@optimizer/gradients_1/optimizer/clip_by_value/Minimum_grad/Shape*
T0*
Tshape0
�
Eoptimizer/gradients_1/optimizer/clip_by_value/Minimum_grad/SelectV2_1SelectV2Doptimizer/gradients_1/optimizer/clip_by_value/Minimum_grad/LessEqual@optimizer/gradients_1/optimizer/clip_by_value/Minimum_grad/zerosKoptimizer/gradients_1/optimizer/clip_by_value_grad/tuple/control_dependency*
T0
�
@optimizer/gradients_1/optimizer/clip_by_value/Minimum_grad/Sum_1SumEoptimizer/gradients_1/optimizer/clip_by_value/Minimum_grad/SelectV2_1Roptimizer/gradients_1/optimizer/clip_by_value/Minimum_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
Doptimizer/gradients_1/optimizer/clip_by_value/Minimum_grad/Reshape_1Reshape@optimizer/gradients_1/optimizer/clip_by_value/Minimum_grad/Sum_1Boptimizer/gradients_1/optimizer/clip_by_value/Minimum_grad/Shape_1*
T0*
Tshape0
�
Koptimizer/gradients_1/optimizer/clip_by_value/Minimum_grad/tuple/group_depsNoOpC^optimizer/gradients_1/optimizer/clip_by_value/Minimum_grad/ReshapeE^optimizer/gradients_1/optimizer/clip_by_value/Minimum_grad/Reshape_1
�
Soptimizer/gradients_1/optimizer/clip_by_value/Minimum_grad/tuple/control_dependencyIdentityBoptimizer/gradients_1/optimizer/clip_by_value/Minimum_grad/ReshapeL^optimizer/gradients_1/optimizer/clip_by_value/Minimum_grad/tuple/group_deps*
T0*U
_classK
IGloc:@optimizer/gradients_1/optimizer/clip_by_value/Minimum_grad/Reshape
�
Uoptimizer/gradients_1/optimizer/clip_by_value/Minimum_grad/tuple/control_dependency_1IdentityDoptimizer/gradients_1/optimizer/clip_by_value/Minimum_grad/Reshape_1L^optimizer/gradients_1/optimizer/clip_by_value/Minimum_grad/tuple/group_deps*
T0*W
_classM
KIloc:@optimizer/gradients_1/optimizer/clip_by_value/Minimum_grad/Reshape_1
u
Boptimizer/gradients_1/optimizer/clip_by_value_1/Minimum_grad/ShapeShapeoptimizer/sub_1*
T0*
out_type0
m
Doptimizer/gradients_1/optimizer/clip_by_value_1/Minimum_grad/Shape_1Const*
dtype0*
valueB 
�
Doptimizer/gradients_1/optimizer/clip_by_value_1/Minimum_grad/Shape_2ShapeMoptimizer/gradients_1/optimizer/clip_by_value_1_grad/tuple/control_dependency*
T0*
out_type0
u
Hoptimizer/gradients_1/optimizer/clip_by_value_1/Minimum_grad/zeros/ConstConst*
dtype0*
valueB
 *    
�
Boptimizer/gradients_1/optimizer/clip_by_value_1/Minimum_grad/zerosFillDoptimizer/gradients_1/optimizer/clip_by_value_1/Minimum_grad/Shape_2Hoptimizer/gradients_1/optimizer/clip_by_value_1/Minimum_grad/zeros/Const*
T0*

index_type0
�
Foptimizer/gradients_1/optimizer/clip_by_value_1/Minimum_grad/LessEqual	LessEqualoptimizer/sub_1optimizer/Variable_1/read*
T0
�
Roptimizer/gradients_1/optimizer/clip_by_value_1/Minimum_grad/BroadcastGradientArgsBroadcastGradientArgsBoptimizer/gradients_1/optimizer/clip_by_value_1/Minimum_grad/ShapeDoptimizer/gradients_1/optimizer/clip_by_value_1/Minimum_grad/Shape_1*
T0
�
Eoptimizer/gradients_1/optimizer/clip_by_value_1/Minimum_grad/SelectV2SelectV2Foptimizer/gradients_1/optimizer/clip_by_value_1/Minimum_grad/LessEqualMoptimizer/gradients_1/optimizer/clip_by_value_1_grad/tuple/control_dependencyBoptimizer/gradients_1/optimizer/clip_by_value_1/Minimum_grad/zeros*
T0
�
@optimizer/gradients_1/optimizer/clip_by_value_1/Minimum_grad/SumSumEoptimizer/gradients_1/optimizer/clip_by_value_1/Minimum_grad/SelectV2Roptimizer/gradients_1/optimizer/clip_by_value_1/Minimum_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
Doptimizer/gradients_1/optimizer/clip_by_value_1/Minimum_grad/ReshapeReshape@optimizer/gradients_1/optimizer/clip_by_value_1/Minimum_grad/SumBoptimizer/gradients_1/optimizer/clip_by_value_1/Minimum_grad/Shape*
T0*
Tshape0
�
Goptimizer/gradients_1/optimizer/clip_by_value_1/Minimum_grad/SelectV2_1SelectV2Foptimizer/gradients_1/optimizer/clip_by_value_1/Minimum_grad/LessEqualBoptimizer/gradients_1/optimizer/clip_by_value_1/Minimum_grad/zerosMoptimizer/gradients_1/optimizer/clip_by_value_1_grad/tuple/control_dependency*
T0
�
Boptimizer/gradients_1/optimizer/clip_by_value_1/Minimum_grad/Sum_1SumGoptimizer/gradients_1/optimizer/clip_by_value_1/Minimum_grad/SelectV2_1Toptimizer/gradients_1/optimizer/clip_by_value_1/Minimum_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
Foptimizer/gradients_1/optimizer/clip_by_value_1/Minimum_grad/Reshape_1ReshapeBoptimizer/gradients_1/optimizer/clip_by_value_1/Minimum_grad/Sum_1Doptimizer/gradients_1/optimizer/clip_by_value_1/Minimum_grad/Shape_1*
T0*
Tshape0
�
Moptimizer/gradients_1/optimizer/clip_by_value_1/Minimum_grad/tuple/group_depsNoOpE^optimizer/gradients_1/optimizer/clip_by_value_1/Minimum_grad/ReshapeG^optimizer/gradients_1/optimizer/clip_by_value_1/Minimum_grad/Reshape_1
�
Uoptimizer/gradients_1/optimizer/clip_by_value_1/Minimum_grad/tuple/control_dependencyIdentityDoptimizer/gradients_1/optimizer/clip_by_value_1/Minimum_grad/ReshapeN^optimizer/gradients_1/optimizer/clip_by_value_1/Minimum_grad/tuple/group_deps*
T0*W
_classM
KIloc:@optimizer/gradients_1/optimizer/clip_by_value_1/Minimum_grad/Reshape
�
Woptimizer/gradients_1/optimizer/clip_by_value_1/Minimum_grad/tuple/control_dependency_1IdentityFoptimizer/gradients_1/optimizer/clip_by_value_1/Minimum_grad/Reshape_1N^optimizer/gradients_1/optimizer/clip_by_value_1/Minimum_grad/tuple/group_deps*
T0*Y
_classO
MKloc:@optimizer/gradients_1/optimizer/clip_by_value_1/Minimum_grad/Reshape_1
�
3optimizer/gradients_1/policy_1/stack_1_grad/unstackUnpack5optimizer/gradients_1/policy_1/Sum_3_grad/BroadcastTo*
T0*

axis*	
num
z
<optimizer/gradients_1/policy_1/stack_1_grad/tuple/group_depsNoOp4^optimizer/gradients_1/policy_1/stack_1_grad/unstack
�
Doptimizer/gradients_1/policy_1/stack_1_grad/tuple/control_dependencyIdentity3optimizer/gradients_1/policy_1/stack_1_grad/unstack=^optimizer/gradients_1/policy_1/stack_1_grad/tuple/group_deps*
T0*F
_class<
:8loc:@optimizer/gradients_1/policy_1/stack_1_grad/unstack
�
Foptimizer/gradients_1/policy_1/stack_1_grad/tuple/control_dependency_1Identity5optimizer/gradients_1/policy_1/stack_1_grad/unstack:1=^optimizer/gradients_1/policy_1/stack_1_grad/tuple/group_deps*
T0*F
_class<
:8loc:@optimizer/gradients_1/policy_1/stack_1_grad/unstack
a
.optimizer/gradients_1/optimizer/sub_grad/ShapeShapeoptimizer/Sum_5*
T0*
out_type0
v
0optimizer/gradients_1/optimizer/sub_grad/Shape_1Shape"optimizer/extrinsic_value_estimate*
T0*
out_type0
�
>optimizer/gradients_1/optimizer/sub_grad/BroadcastGradientArgsBroadcastGradientArgs.optimizer/gradients_1/optimizer/sub_grad/Shape0optimizer/gradients_1/optimizer/sub_grad/Shape_1*
T0
�
,optimizer/gradients_1/optimizer/sub_grad/SumSumSoptimizer/gradients_1/optimizer/clip_by_value/Minimum_grad/tuple/control_dependency>optimizer/gradients_1/optimizer/sub_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
0optimizer/gradients_1/optimizer/sub_grad/ReshapeReshape,optimizer/gradients_1/optimizer/sub_grad/Sum.optimizer/gradients_1/optimizer/sub_grad/Shape*
T0*
Tshape0
�
,optimizer/gradients_1/optimizer/sub_grad/NegNegSoptimizer/gradients_1/optimizer/clip_by_value/Minimum_grad/tuple/control_dependency*
T0
�
.optimizer/gradients_1/optimizer/sub_grad/Sum_1Sum,optimizer/gradients_1/optimizer/sub_grad/Neg@optimizer/gradients_1/optimizer/sub_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
2optimizer/gradients_1/optimizer/sub_grad/Reshape_1Reshape.optimizer/gradients_1/optimizer/sub_grad/Sum_10optimizer/gradients_1/optimizer/sub_grad/Shape_1*
T0*
Tshape0
�
9optimizer/gradients_1/optimizer/sub_grad/tuple/group_depsNoOp1^optimizer/gradients_1/optimizer/sub_grad/Reshape3^optimizer/gradients_1/optimizer/sub_grad/Reshape_1
�
Aoptimizer/gradients_1/optimizer/sub_grad/tuple/control_dependencyIdentity0optimizer/gradients_1/optimizer/sub_grad/Reshape:^optimizer/gradients_1/optimizer/sub_grad/tuple/group_deps*
T0*C
_class9
75loc:@optimizer/gradients_1/optimizer/sub_grad/Reshape
�
Coptimizer/gradients_1/optimizer/sub_grad/tuple/control_dependency_1Identity2optimizer/gradients_1/optimizer/sub_grad/Reshape_1:^optimizer/gradients_1/optimizer/sub_grad/tuple/group_deps*
T0*E
_class;
97loc:@optimizer/gradients_1/optimizer/sub_grad/Reshape_1
c
0optimizer/gradients_1/optimizer/sub_1_grad/ShapeShapeoptimizer/Sum_7*
T0*
out_type0
x
2optimizer/gradients_1/optimizer/sub_1_grad/Shape_1Shape"optimizer/curiosity_value_estimate*
T0*
out_type0
�
@optimizer/gradients_1/optimizer/sub_1_grad/BroadcastGradientArgsBroadcastGradientArgs0optimizer/gradients_1/optimizer/sub_1_grad/Shape2optimizer/gradients_1/optimizer/sub_1_grad/Shape_1*
T0
�
.optimizer/gradients_1/optimizer/sub_1_grad/SumSumUoptimizer/gradients_1/optimizer/clip_by_value_1/Minimum_grad/tuple/control_dependency@optimizer/gradients_1/optimizer/sub_1_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
2optimizer/gradients_1/optimizer/sub_1_grad/ReshapeReshape.optimizer/gradients_1/optimizer/sub_1_grad/Sum0optimizer/gradients_1/optimizer/sub_1_grad/Shape*
T0*
Tshape0
�
.optimizer/gradients_1/optimizer/sub_1_grad/NegNegUoptimizer/gradients_1/optimizer/clip_by_value_1/Minimum_grad/tuple/control_dependency*
T0
�
0optimizer/gradients_1/optimizer/sub_1_grad/Sum_1Sum.optimizer/gradients_1/optimizer/sub_1_grad/NegBoptimizer/gradients_1/optimizer/sub_1_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
4optimizer/gradients_1/optimizer/sub_1_grad/Reshape_1Reshape0optimizer/gradients_1/optimizer/sub_1_grad/Sum_12optimizer/gradients_1/optimizer/sub_1_grad/Shape_1*
T0*
Tshape0
�
;optimizer/gradients_1/optimizer/sub_1_grad/tuple/group_depsNoOp3^optimizer/gradients_1/optimizer/sub_1_grad/Reshape5^optimizer/gradients_1/optimizer/sub_1_grad/Reshape_1
�
Coptimizer/gradients_1/optimizer/sub_1_grad/tuple/control_dependencyIdentity2optimizer/gradients_1/optimizer/sub_1_grad/Reshape<^optimizer/gradients_1/optimizer/sub_1_grad/tuple/group_deps*
T0*E
_class;
97loc:@optimizer/gradients_1/optimizer/sub_1_grad/Reshape
�
Eoptimizer/gradients_1/optimizer/sub_1_grad/tuple/control_dependency_1Identity4optimizer/gradients_1/optimizer/sub_1_grad/Reshape_1<^optimizer/gradients_1/optimizer/sub_1_grad/tuple/group_deps*
T0*G
_class=
;9loc:@optimizer/gradients_1/optimizer/sub_1_grad/Reshape_1
�
+optimizer/gradients_1/policy_1/Neg_grad/NegNegDoptimizer/gradients_1/policy_1/stack_1_grad/tuple/control_dependency*
T0
�
-optimizer/gradients_1/policy_1/Neg_1_grad/NegNegFoptimizer/gradients_1/policy_1/stack_1_grad/tuple/control_dependency_1*
T0
u
0optimizer/gradients_1/optimizer/Sum_5_grad/ShapeShape!optimizer/extrinsic_value/BiasAdd*
T0*
out_type0
�
/optimizer/gradients_1/optimizer/Sum_5_grad/SizeConst*C
_class9
75loc:@optimizer/gradients_1/optimizer/Sum_5_grad/Shape*
dtype0*
value	B :
�
.optimizer/gradients_1/optimizer/Sum_5_grad/addAddV2!optimizer/Sum_5/reduction_indices/optimizer/gradients_1/optimizer/Sum_5_grad/Size*
T0*C
_class9
75loc:@optimizer/gradients_1/optimizer/Sum_5_grad/Shape
�
.optimizer/gradients_1/optimizer/Sum_5_grad/modFloorMod.optimizer/gradients_1/optimizer/Sum_5_grad/add/optimizer/gradients_1/optimizer/Sum_5_grad/Size*
T0*C
_class9
75loc:@optimizer/gradients_1/optimizer/Sum_5_grad/Shape
�
2optimizer/gradients_1/optimizer/Sum_5_grad/Shape_1Const*C
_class9
75loc:@optimizer/gradients_1/optimizer/Sum_5_grad/Shape*
dtype0*
valueB 
�
6optimizer/gradients_1/optimizer/Sum_5_grad/range/startConst*C
_class9
75loc:@optimizer/gradients_1/optimizer/Sum_5_grad/Shape*
dtype0*
value	B : 
�
6optimizer/gradients_1/optimizer/Sum_5_grad/range/deltaConst*C
_class9
75loc:@optimizer/gradients_1/optimizer/Sum_5_grad/Shape*
dtype0*
value	B :
�
0optimizer/gradients_1/optimizer/Sum_5_grad/rangeRange6optimizer/gradients_1/optimizer/Sum_5_grad/range/start/optimizer/gradients_1/optimizer/Sum_5_grad/Size6optimizer/gradients_1/optimizer/Sum_5_grad/range/delta*

Tidx0*C
_class9
75loc:@optimizer/gradients_1/optimizer/Sum_5_grad/Shape
�
5optimizer/gradients_1/optimizer/Sum_5_grad/Fill/valueConst*C
_class9
75loc:@optimizer/gradients_1/optimizer/Sum_5_grad/Shape*
dtype0*
value	B :
�
/optimizer/gradients_1/optimizer/Sum_5_grad/FillFill2optimizer/gradients_1/optimizer/Sum_5_grad/Shape_15optimizer/gradients_1/optimizer/Sum_5_grad/Fill/value*
T0*C
_class9
75loc:@optimizer/gradients_1/optimizer/Sum_5_grad/Shape*

index_type0
�
8optimizer/gradients_1/optimizer/Sum_5_grad/DynamicStitchDynamicStitch0optimizer/gradients_1/optimizer/Sum_5_grad/range.optimizer/gradients_1/optimizer/Sum_5_grad/mod0optimizer/gradients_1/optimizer/Sum_5_grad/Shape/optimizer/gradients_1/optimizer/Sum_5_grad/Fill*
N*
T0*C
_class9
75loc:@optimizer/gradients_1/optimizer/Sum_5_grad/Shape
�
2optimizer/gradients_1/optimizer/Sum_5_grad/ReshapeReshapeAoptimizer/gradients_1/optimizer/sub_grad/tuple/control_dependency8optimizer/gradients_1/optimizer/Sum_5_grad/DynamicStitch*
T0*
Tshape0
�
6optimizer/gradients_1/optimizer/Sum_5_grad/BroadcastToBroadcastTo2optimizer/gradients_1/optimizer/Sum_5_grad/Reshape0optimizer/gradients_1/optimizer/Sum_5_grad/Shape*
T0*

Tidx0
u
0optimizer/gradients_1/optimizer/Sum_7_grad/ShapeShape!optimizer/curiosity_value/BiasAdd*
T0*
out_type0
�
/optimizer/gradients_1/optimizer/Sum_7_grad/SizeConst*C
_class9
75loc:@optimizer/gradients_1/optimizer/Sum_7_grad/Shape*
dtype0*
value	B :
�
.optimizer/gradients_1/optimizer/Sum_7_grad/addAddV2!optimizer/Sum_7/reduction_indices/optimizer/gradients_1/optimizer/Sum_7_grad/Size*
T0*C
_class9
75loc:@optimizer/gradients_1/optimizer/Sum_7_grad/Shape
�
.optimizer/gradients_1/optimizer/Sum_7_grad/modFloorMod.optimizer/gradients_1/optimizer/Sum_7_grad/add/optimizer/gradients_1/optimizer/Sum_7_grad/Size*
T0*C
_class9
75loc:@optimizer/gradients_1/optimizer/Sum_7_grad/Shape
�
2optimizer/gradients_1/optimizer/Sum_7_grad/Shape_1Const*C
_class9
75loc:@optimizer/gradients_1/optimizer/Sum_7_grad/Shape*
dtype0*
valueB 
�
6optimizer/gradients_1/optimizer/Sum_7_grad/range/startConst*C
_class9
75loc:@optimizer/gradients_1/optimizer/Sum_7_grad/Shape*
dtype0*
value	B : 
�
6optimizer/gradients_1/optimizer/Sum_7_grad/range/deltaConst*C
_class9
75loc:@optimizer/gradients_1/optimizer/Sum_7_grad/Shape*
dtype0*
value	B :
�
0optimizer/gradients_1/optimizer/Sum_7_grad/rangeRange6optimizer/gradients_1/optimizer/Sum_7_grad/range/start/optimizer/gradients_1/optimizer/Sum_7_grad/Size6optimizer/gradients_1/optimizer/Sum_7_grad/range/delta*

Tidx0*C
_class9
75loc:@optimizer/gradients_1/optimizer/Sum_7_grad/Shape
�
5optimizer/gradients_1/optimizer/Sum_7_grad/Fill/valueConst*C
_class9
75loc:@optimizer/gradients_1/optimizer/Sum_7_grad/Shape*
dtype0*
value	B :
�
/optimizer/gradients_1/optimizer/Sum_7_grad/FillFill2optimizer/gradients_1/optimizer/Sum_7_grad/Shape_15optimizer/gradients_1/optimizer/Sum_7_grad/Fill/value*
T0*C
_class9
75loc:@optimizer/gradients_1/optimizer/Sum_7_grad/Shape*

index_type0
�
8optimizer/gradients_1/optimizer/Sum_7_grad/DynamicStitchDynamicStitch0optimizer/gradients_1/optimizer/Sum_7_grad/range.optimizer/gradients_1/optimizer/Sum_7_grad/mod0optimizer/gradients_1/optimizer/Sum_7_grad/Shape/optimizer/gradients_1/optimizer/Sum_7_grad/Fill*
N*
T0*C
_class9
75loc:@optimizer/gradients_1/optimizer/Sum_7_grad/Shape
�
2optimizer/gradients_1/optimizer/Sum_7_grad/ReshapeReshapeCoptimizer/gradients_1/optimizer/sub_1_grad/tuple/control_dependency8optimizer/gradients_1/optimizer/Sum_7_grad/DynamicStitch*
T0*
Tshape0
�
6optimizer/gradients_1/optimizer/Sum_7_grad/BroadcastToBroadcastTo2optimizer/gradients_1/optimizer/Sum_7_grad/Reshape0optimizer/gradients_1/optimizer/Sum_7_grad/Shape*
T0*

Tidx0
�
Woptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_2/Reshape_2_grad/ShapeShape,policy_1/softmax_cross_entropy_with_logits_2*
T0*
out_type0
�
Yoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_2/Reshape_2_grad/ReshapeReshape+optimizer/gradients_1/policy_1/Neg_grad/NegWoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_2/Reshape_2_grad/Shape*
T0*
Tshape0
�
Woptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_3/Reshape_2_grad/ShapeShape,policy_1/softmax_cross_entropy_with_logits_3*
T0*
out_type0
�
Yoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_3/Reshape_2_grad/ReshapeReshape-optimizer/gradients_1/policy_1/Neg_1_grad/NegWoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_3/Reshape_2_grad/Shape*
T0*
Tshape0
�
optimizer/gradients_1/AddN_1AddN6optimizer/gradients_1/optimizer/Sum_6_grad/BroadcastTo6optimizer/gradients_1/optimizer/Sum_5_grad/BroadcastTo*
N*
T0*I
_class?
=;loc:@optimizer/gradients_1/optimizer/Sum_6_grad/BroadcastTo
�
Hoptimizer/gradients_1/optimizer/extrinsic_value/BiasAdd_grad/BiasAddGradBiasAddGradoptimizer/gradients_1/AddN_1*
T0*
data_formatNHWC
�
Moptimizer/gradients_1/optimizer/extrinsic_value/BiasAdd_grad/tuple/group_depsNoOp^optimizer/gradients_1/AddN_1I^optimizer/gradients_1/optimizer/extrinsic_value/BiasAdd_grad/BiasAddGrad
�
Uoptimizer/gradients_1/optimizer/extrinsic_value/BiasAdd_grad/tuple/control_dependencyIdentityoptimizer/gradients_1/AddN_1N^optimizer/gradients_1/optimizer/extrinsic_value/BiasAdd_grad/tuple/group_deps*
T0*I
_class?
=;loc:@optimizer/gradients_1/optimizer/Sum_6_grad/BroadcastTo
�
Woptimizer/gradients_1/optimizer/extrinsic_value/BiasAdd_grad/tuple/control_dependency_1IdentityHoptimizer/gradients_1/optimizer/extrinsic_value/BiasAdd_grad/BiasAddGradN^optimizer/gradients_1/optimizer/extrinsic_value/BiasAdd_grad/tuple/group_deps*
T0*[
_classQ
OMloc:@optimizer/gradients_1/optimizer/extrinsic_value/BiasAdd_grad/BiasAddGrad
�
optimizer/gradients_1/AddN_2AddN6optimizer/gradients_1/optimizer/Sum_8_grad/BroadcastTo6optimizer/gradients_1/optimizer/Sum_7_grad/BroadcastTo*
N*
T0*I
_class?
=;loc:@optimizer/gradients_1/optimizer/Sum_8_grad/BroadcastTo
�
Hoptimizer/gradients_1/optimizer/curiosity_value/BiasAdd_grad/BiasAddGradBiasAddGradoptimizer/gradients_1/AddN_2*
T0*
data_formatNHWC
�
Moptimizer/gradients_1/optimizer/curiosity_value/BiasAdd_grad/tuple/group_depsNoOp^optimizer/gradients_1/AddN_2I^optimizer/gradients_1/optimizer/curiosity_value/BiasAdd_grad/BiasAddGrad
�
Uoptimizer/gradients_1/optimizer/curiosity_value/BiasAdd_grad/tuple/control_dependencyIdentityoptimizer/gradients_1/AddN_2N^optimizer/gradients_1/optimizer/curiosity_value/BiasAdd_grad/tuple/group_deps*
T0*I
_class?
=;loc:@optimizer/gradients_1/optimizer/Sum_8_grad/BroadcastTo
�
Woptimizer/gradients_1/optimizer/curiosity_value/BiasAdd_grad/tuple/control_dependency_1IdentityHoptimizer/gradients_1/optimizer/curiosity_value/BiasAdd_grad/BiasAddGradN^optimizer/gradients_1/optimizer/curiosity_value/BiasAdd_grad/tuple/group_deps*
T0*[
_classQ
OMloc:@optimizer/gradients_1/optimizer/curiosity_value/BiasAdd_grad/BiasAddGrad
h
"optimizer/gradients_1/zeros_like_6	ZerosLike.policy_1/softmax_cross_entropy_with_logits_2:1*
T0
�
Voptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_2_grad/ExpandDims/dimConst*
dtype0*
valueB :
���������
�
Roptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_2_grad/ExpandDims
ExpandDimsYoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_2/Reshape_2_grad/ReshapeVoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_2_grad/ExpandDims/dim*
T0*

Tdim0
�
Koptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_2_grad/mulMulRoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_2_grad/ExpandDims.policy_1/softmax_cross_entropy_with_logits_2:1*
T0
�
Roptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_2_grad/LogSoftmax
LogSoftmax4policy_1/softmax_cross_entropy_with_logits_2/Reshape*
T0
�
Koptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_2_grad/NegNegRoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_2_grad/LogSoftmax*
T0
�
Xoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_2_grad/ExpandDims_1/dimConst*
dtype0*
valueB :
���������
�
Toptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_2_grad/ExpandDims_1
ExpandDimsYoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_2/Reshape_2_grad/ReshapeXoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_2_grad/ExpandDims_1/dim*
T0*

Tdim0
�
Moptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_2_grad/mul_1MulToptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_2_grad/ExpandDims_1Koptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_2_grad/Neg*
T0
�
Xoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_2_grad/tuple/group_depsNoOpL^optimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_2_grad/mulN^optimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_2_grad/mul_1
�
`optimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_2_grad/tuple/control_dependencyIdentityKoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_2_grad/mulY^optimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_2_grad/tuple/group_deps*
T0*^
_classT
RPloc:@optimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_2_grad/mul
�
boptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_2_grad/tuple/control_dependency_1IdentityMoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_2_grad/mul_1Y^optimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_2_grad/tuple/group_deps*
T0*`
_classV
TRloc:@optimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_2_grad/mul_1
h
"optimizer/gradients_1/zeros_like_7	ZerosLike.policy_1/softmax_cross_entropy_with_logits_3:1*
T0
�
Voptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_3_grad/ExpandDims/dimConst*
dtype0*
valueB :
���������
�
Roptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_3_grad/ExpandDims
ExpandDimsYoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_3/Reshape_2_grad/ReshapeVoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_3_grad/ExpandDims/dim*
T0*

Tdim0
�
Koptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_3_grad/mulMulRoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_3_grad/ExpandDims.policy_1/softmax_cross_entropy_with_logits_3:1*
T0
�
Roptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_3_grad/LogSoftmax
LogSoftmax4policy_1/softmax_cross_entropy_with_logits_3/Reshape*
T0
�
Koptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_3_grad/NegNegRoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_3_grad/LogSoftmax*
T0
�
Xoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_3_grad/ExpandDims_1/dimConst*
dtype0*
valueB :
���������
�
Toptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_3_grad/ExpandDims_1
ExpandDimsYoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_3/Reshape_2_grad/ReshapeXoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_3_grad/ExpandDims_1/dim*
T0*

Tdim0
�
Moptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_3_grad/mul_1MulToptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_3_grad/ExpandDims_1Koptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_3_grad/Neg*
T0
�
Xoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_3_grad/tuple/group_depsNoOpL^optimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_3_grad/mulN^optimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_3_grad/mul_1
�
`optimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_3_grad/tuple/control_dependencyIdentityKoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_3_grad/mulY^optimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_3_grad/tuple/group_deps*
T0*^
_classT
RPloc:@optimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_3_grad/mul
�
boptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_3_grad/tuple/control_dependency_1IdentityMoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_3_grad/mul_1Y^optimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_3_grad/tuple/group_deps*
T0*`
_classV
TRloc:@optimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_3_grad/mul_1
�
Boptimizer/gradients_1/optimizer/extrinsic_value/MatMul_grad/MatMulMatMulUoptimizer/gradients_1/optimizer/extrinsic_value/BiasAdd_grad/tuple/control_dependency&optimizer//extrinsic_value/kernel/read*
T0*
transpose_a( *
transpose_b(
�
Doptimizer/gradients_1/optimizer/extrinsic_value/MatMul_grad/MatMul_1MatMul#optimizer/main_graph_0/hidden_1/MulUoptimizer/gradients_1/optimizer/extrinsic_value/BiasAdd_grad/tuple/control_dependency*
T0*
transpose_a(*
transpose_b( 
�
Loptimizer/gradients_1/optimizer/extrinsic_value/MatMul_grad/tuple/group_depsNoOpC^optimizer/gradients_1/optimizer/extrinsic_value/MatMul_grad/MatMulE^optimizer/gradients_1/optimizer/extrinsic_value/MatMul_grad/MatMul_1
�
Toptimizer/gradients_1/optimizer/extrinsic_value/MatMul_grad/tuple/control_dependencyIdentityBoptimizer/gradients_1/optimizer/extrinsic_value/MatMul_grad/MatMulM^optimizer/gradients_1/optimizer/extrinsic_value/MatMul_grad/tuple/group_deps*
T0*U
_classK
IGloc:@optimizer/gradients_1/optimizer/extrinsic_value/MatMul_grad/MatMul
�
Voptimizer/gradients_1/optimizer/extrinsic_value/MatMul_grad/tuple/control_dependency_1IdentityDoptimizer/gradients_1/optimizer/extrinsic_value/MatMul_grad/MatMul_1M^optimizer/gradients_1/optimizer/extrinsic_value/MatMul_grad/tuple/group_deps*
T0*W
_classM
KIloc:@optimizer/gradients_1/optimizer/extrinsic_value/MatMul_grad/MatMul_1
�
Boptimizer/gradients_1/optimizer/curiosity_value/MatMul_grad/MatMulMatMulUoptimizer/gradients_1/optimizer/curiosity_value/BiasAdd_grad/tuple/control_dependency&optimizer//curiosity_value/kernel/read*
T0*
transpose_a( *
transpose_b(
�
Doptimizer/gradients_1/optimizer/curiosity_value/MatMul_grad/MatMul_1MatMul#optimizer/main_graph_0/hidden_1/MulUoptimizer/gradients_1/optimizer/curiosity_value/BiasAdd_grad/tuple/control_dependency*
T0*
transpose_a(*
transpose_b( 
�
Loptimizer/gradients_1/optimizer/curiosity_value/MatMul_grad/tuple/group_depsNoOpC^optimizer/gradients_1/optimizer/curiosity_value/MatMul_grad/MatMulE^optimizer/gradients_1/optimizer/curiosity_value/MatMul_grad/MatMul_1
�
Toptimizer/gradients_1/optimizer/curiosity_value/MatMul_grad/tuple/control_dependencyIdentityBoptimizer/gradients_1/optimizer/curiosity_value/MatMul_grad/MatMulM^optimizer/gradients_1/optimizer/curiosity_value/MatMul_grad/tuple/group_deps*
T0*U
_classK
IGloc:@optimizer/gradients_1/optimizer/curiosity_value/MatMul_grad/MatMul
�
Voptimizer/gradients_1/optimizer/curiosity_value/MatMul_grad/tuple/control_dependency_1IdentityDoptimizer/gradients_1/optimizer/curiosity_value/MatMul_grad/MatMul_1M^optimizer/gradients_1/optimizer/curiosity_value/MatMul_grad/tuple/group_deps*
T0*W
_classM
KIloc:@optimizer/gradients_1/optimizer/curiosity_value/MatMul_grad/MatMul_1
�
Uoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_2/Reshape_grad/ShapeShapepolicy_1/strided_slice_9*
T0*
out_type0
�
Woptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_2/Reshape_grad/ReshapeReshape`optimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_2_grad/tuple/control_dependencyUoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_2/Reshape_grad/Shape*
T0*
Tshape0
�
Uoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_3/Reshape_grad/ShapeShapepolicy_1/strided_slice_11*
T0*
out_type0
�
Woptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_3/Reshape_grad/ReshapeReshape`optimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_3_grad/tuple/control_dependencyUoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_3/Reshape_grad/Shape*
T0*
Tshape0
�
optimizer/gradients_1/AddN_3AddNToptimizer/gradients_1/optimizer/extrinsic_value/MatMul_grad/tuple/control_dependencyToptimizer/gradients_1/optimizer/curiosity_value/MatMul_grad/tuple/control_dependency*
N*
T0*U
_classK
IGloc:@optimizer/gradients_1/optimizer/extrinsic_value/MatMul_grad/MatMul
�
Doptimizer/gradients_1/optimizer/main_graph_0/hidden_1/Mul_grad/ShapeShape'optimizer/main_graph_0/hidden_1/BiasAdd*
T0*
out_type0
�
Foptimizer/gradients_1/optimizer/main_graph_0/hidden_1/Mul_grad/Shape_1Shape'optimizer/main_graph_0/hidden_1/Sigmoid*
T0*
out_type0
�
Toptimizer/gradients_1/optimizer/main_graph_0/hidden_1/Mul_grad/BroadcastGradientArgsBroadcastGradientArgsDoptimizer/gradients_1/optimizer/main_graph_0/hidden_1/Mul_grad/ShapeFoptimizer/gradients_1/optimizer/main_graph_0/hidden_1/Mul_grad/Shape_1*
T0
�
Boptimizer/gradients_1/optimizer/main_graph_0/hidden_1/Mul_grad/MulMuloptimizer/gradients_1/AddN_3'optimizer/main_graph_0/hidden_1/Sigmoid*
T0
�
Boptimizer/gradients_1/optimizer/main_graph_0/hidden_1/Mul_grad/SumSumBoptimizer/gradients_1/optimizer/main_graph_0/hidden_1/Mul_grad/MulToptimizer/gradients_1/optimizer/main_graph_0/hidden_1/Mul_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
Foptimizer/gradients_1/optimizer/main_graph_0/hidden_1/Mul_grad/ReshapeReshapeBoptimizer/gradients_1/optimizer/main_graph_0/hidden_1/Mul_grad/SumDoptimizer/gradients_1/optimizer/main_graph_0/hidden_1/Mul_grad/Shape*
T0*
Tshape0
�
Doptimizer/gradients_1/optimizer/main_graph_0/hidden_1/Mul_grad/Mul_1Mul'optimizer/main_graph_0/hidden_1/BiasAddoptimizer/gradients_1/AddN_3*
T0
�
Doptimizer/gradients_1/optimizer/main_graph_0/hidden_1/Mul_grad/Sum_1SumDoptimizer/gradients_1/optimizer/main_graph_0/hidden_1/Mul_grad/Mul_1Voptimizer/gradients_1/optimizer/main_graph_0/hidden_1/Mul_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
Hoptimizer/gradients_1/optimizer/main_graph_0/hidden_1/Mul_grad/Reshape_1ReshapeDoptimizer/gradients_1/optimizer/main_graph_0/hidden_1/Mul_grad/Sum_1Foptimizer/gradients_1/optimizer/main_graph_0/hidden_1/Mul_grad/Shape_1*
T0*
Tshape0
�
Ooptimizer/gradients_1/optimizer/main_graph_0/hidden_1/Mul_grad/tuple/group_depsNoOpG^optimizer/gradients_1/optimizer/main_graph_0/hidden_1/Mul_grad/ReshapeI^optimizer/gradients_1/optimizer/main_graph_0/hidden_1/Mul_grad/Reshape_1
�
Woptimizer/gradients_1/optimizer/main_graph_0/hidden_1/Mul_grad/tuple/control_dependencyIdentityFoptimizer/gradients_1/optimizer/main_graph_0/hidden_1/Mul_grad/ReshapeP^optimizer/gradients_1/optimizer/main_graph_0/hidden_1/Mul_grad/tuple/group_deps*
T0*Y
_classO
MKloc:@optimizer/gradients_1/optimizer/main_graph_0/hidden_1/Mul_grad/Reshape
�
Yoptimizer/gradients_1/optimizer/main_graph_0/hidden_1/Mul_grad/tuple/control_dependency_1IdentityHoptimizer/gradients_1/optimizer/main_graph_0/hidden_1/Mul_grad/Reshape_1P^optimizer/gradients_1/optimizer/main_graph_0/hidden_1/Mul_grad/tuple/group_deps*
T0*[
_classQ
OMloc:@optimizer/gradients_1/optimizer/main_graph_0/hidden_1/Mul_grad/Reshape_1
n
9optimizer/gradients_1/policy_1/strided_slice_9_grad/ShapeShapepolicy_1/concat_2*
T0*
out_type0

Joptimizer/gradients_1/policy_1/strided_slice_9_grad/StridedSliceGrad/beginConst*
dtype0*
valueB"        
}
Hoptimizer/gradients_1/policy_1/strided_slice_9_grad/StridedSliceGrad/endConst*
dtype0*
valueB"       
�
Loptimizer/gradients_1/policy_1/strided_slice_9_grad/StridedSliceGrad/stridesConst*
dtype0*
valueB"      
�
Doptimizer/gradients_1/policy_1/strided_slice_9_grad/StridedSliceGradStridedSliceGrad9optimizer/gradients_1/policy_1/strided_slice_9_grad/ShapeJoptimizer/gradients_1/policy_1/strided_slice_9_grad/StridedSliceGrad/beginHoptimizer/gradients_1/policy_1/strided_slice_9_grad/StridedSliceGrad/endLoptimizer/gradients_1/policy_1/strided_slice_9_grad/StridedSliceGrad/stridesWoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_2/Reshape_grad/Reshape*
Index0*
T0*

begin_mask*
ellipsis_mask *
end_mask*
new_axis_mask *
shrink_axis_mask 
o
:optimizer/gradients_1/policy_1/strided_slice_11_grad/ShapeShapepolicy_1/concat_2*
T0*
out_type0
�
Koptimizer/gradients_1/policy_1/strided_slice_11_grad/StridedSliceGrad/beginConst*
dtype0*
valueB"       
~
Ioptimizer/gradients_1/policy_1/strided_slice_11_grad/StridedSliceGrad/endConst*
dtype0*
valueB"       
�
Moptimizer/gradients_1/policy_1/strided_slice_11_grad/StridedSliceGrad/stridesConst*
dtype0*
valueB"      
�
Eoptimizer/gradients_1/policy_1/strided_slice_11_grad/StridedSliceGradStridedSliceGrad:optimizer/gradients_1/policy_1/strided_slice_11_grad/ShapeKoptimizer/gradients_1/policy_1/strided_slice_11_grad/StridedSliceGrad/beginIoptimizer/gradients_1/policy_1/strided_slice_11_grad/StridedSliceGrad/endMoptimizer/gradients_1/policy_1/strided_slice_11_grad/StridedSliceGrad/stridesWoptimizer/gradients_1/policy_1/softmax_cross_entropy_with_logits_3/Reshape_grad/Reshape*
Index0*
T0*

begin_mask*
ellipsis_mask *
end_mask*
new_axis_mask *
shrink_axis_mask 
�
Noptimizer/gradients_1/optimizer/main_graph_0/hidden_1/Sigmoid_grad/SigmoidGradSigmoidGrad'optimizer/main_graph_0/hidden_1/SigmoidYoptimizer/gradients_1/optimizer/main_graph_0/hidden_1/Mul_grad/tuple/control_dependency_1*
T0
�
optimizer/gradients_1/AddN_4AddNDoptimizer/gradients_1/policy_1/strided_slice_5_grad/StridedSliceGradDoptimizer/gradients_1/policy_1/strided_slice_7_grad/StridedSliceGradDoptimizer/gradients_1/policy_1/strided_slice_4_grad/StridedSliceGradDoptimizer/gradients_1/policy_1/strided_slice_6_grad/StridedSliceGradDoptimizer/gradients_1/policy_1/strided_slice_9_grad/StridedSliceGradEoptimizer/gradients_1/policy_1/strided_slice_11_grad/StridedSliceGrad*
N*
T0*W
_classM
KIloc:@optimizer/gradients_1/policy_1/strided_slice_5_grad/StridedSliceGrad
[
1optimizer/gradients_1/policy_1/concat_2_grad/RankConst*
dtype0*
value	B :
�
0optimizer/gradients_1/policy_1/concat_2_grad/modFloorModpolicy_1/concat_2/axis1optimizer/gradients_1/policy_1/concat_2_grad/Rank*
T0
d
2optimizer/gradients_1/policy_1/concat_2_grad/ShapeShapepolicy_1/Log_2*
T0*
out_type0

3optimizer/gradients_1/policy_1/concat_2_grad/ShapeNShapeNpolicy_1/Log_2policy_1/Log_3*
N*
T0*
out_type0
�
9optimizer/gradients_1/policy_1/concat_2_grad/ConcatOffsetConcatOffset0optimizer/gradients_1/policy_1/concat_2_grad/mod3optimizer/gradients_1/policy_1/concat_2_grad/ShapeN5optimizer/gradients_1/policy_1/concat_2_grad/ShapeN:1*
N
�
2optimizer/gradients_1/policy_1/concat_2_grad/SliceSliceoptimizer/gradients_1/AddN_49optimizer/gradients_1/policy_1/concat_2_grad/ConcatOffset3optimizer/gradients_1/policy_1/concat_2_grad/ShapeN*
Index0*
T0
�
4optimizer/gradients_1/policy_1/concat_2_grad/Slice_1Sliceoptimizer/gradients_1/AddN_4;optimizer/gradients_1/policy_1/concat_2_grad/ConcatOffset:15optimizer/gradients_1/policy_1/concat_2_grad/ShapeN:1*
Index0*
T0
�
=optimizer/gradients_1/policy_1/concat_2_grad/tuple/group_depsNoOp3^optimizer/gradients_1/policy_1/concat_2_grad/Slice5^optimizer/gradients_1/policy_1/concat_2_grad/Slice_1
�
Eoptimizer/gradients_1/policy_1/concat_2_grad/tuple/control_dependencyIdentity2optimizer/gradients_1/policy_1/concat_2_grad/Slice>^optimizer/gradients_1/policy_1/concat_2_grad/tuple/group_deps*
T0*E
_class;
97loc:@optimizer/gradients_1/policy_1/concat_2_grad/Slice
�
Goptimizer/gradients_1/policy_1/concat_2_grad/tuple/control_dependency_1Identity4optimizer/gradients_1/policy_1/concat_2_grad/Slice_1>^optimizer/gradients_1/policy_1/concat_2_grad/tuple/group_deps*
T0*G
_class=
;9loc:@optimizer/gradients_1/policy_1/concat_2_grad/Slice_1
�
optimizer/gradients_1/AddN_5AddNWoptimizer/gradients_1/optimizer/main_graph_0/hidden_1/Mul_grad/tuple/control_dependencyNoptimizer/gradients_1/optimizer/main_graph_0/hidden_1/Sigmoid_grad/SigmoidGrad*
N*
T0*Y
_classO
MKloc:@optimizer/gradients_1/optimizer/main_graph_0/hidden_1/Mul_grad/Reshape
�
Noptimizer/gradients_1/optimizer/main_graph_0/hidden_1/BiasAdd_grad/BiasAddGradBiasAddGradoptimizer/gradients_1/AddN_5*
T0*
data_formatNHWC
�
Soptimizer/gradients_1/optimizer/main_graph_0/hidden_1/BiasAdd_grad/tuple/group_depsNoOp^optimizer/gradients_1/AddN_5O^optimizer/gradients_1/optimizer/main_graph_0/hidden_1/BiasAdd_grad/BiasAddGrad
�
[optimizer/gradients_1/optimizer/main_graph_0/hidden_1/BiasAdd_grad/tuple/control_dependencyIdentityoptimizer/gradients_1/AddN_5T^optimizer/gradients_1/optimizer/main_graph_0/hidden_1/BiasAdd_grad/tuple/group_deps*
T0*Y
_classO
MKloc:@optimizer/gradients_1/optimizer/main_graph_0/hidden_1/Mul_grad/Reshape
�
]optimizer/gradients_1/optimizer/main_graph_0/hidden_1/BiasAdd_grad/tuple/control_dependency_1IdentityNoptimizer/gradients_1/optimizer/main_graph_0/hidden_1/BiasAdd_grad/BiasAddGradT^optimizer/gradients_1/optimizer/main_graph_0/hidden_1/BiasAdd_grad/tuple/group_deps*
T0*a
_classW
USloc:@optimizer/gradients_1/optimizer/main_graph_0/hidden_1/BiasAdd_grad/BiasAddGrad
�
4optimizer/gradients_1/policy_1/Log_2_grad/Reciprocal
Reciprocalpolicy_1/add_4F^optimizer/gradients_1/policy_1/concat_2_grad/tuple/control_dependency*
T0
�
-optimizer/gradients_1/policy_1/Log_2_grad/mulMulEoptimizer/gradients_1/policy_1/concat_2_grad/tuple/control_dependency4optimizer/gradients_1/policy_1/Log_2_grad/Reciprocal*
T0
�
4optimizer/gradients_1/policy_1/Log_3_grad/Reciprocal
Reciprocalpolicy_1/add_5H^optimizer/gradients_1/policy_1/concat_2_grad/tuple/control_dependency_1*
T0
�
-optimizer/gradients_1/policy_1/Log_3_grad/mulMulGoptimizer/gradients_1/policy_1/concat_2_grad/tuple/control_dependency_14optimizer/gradients_1/policy_1/Log_3_grad/Reciprocal*
T0
�
Hoptimizer/gradients_1/optimizer/main_graph_0/hidden_1/MatMul_grad/MatMulMatMul[optimizer/gradients_1/optimizer/main_graph_0/hidden_1/BiasAdd_grad/tuple/control_dependency,optimizer//main_graph_0/hidden_1/kernel/read*
T0*
transpose_a( *
transpose_b(
�
Joptimizer/gradients_1/optimizer/main_graph_0/hidden_1/MatMul_grad/MatMul_1MatMul#optimizer/main_graph_0/hidden_0/Mul[optimizer/gradients_1/optimizer/main_graph_0/hidden_1/BiasAdd_grad/tuple/control_dependency*
T0*
transpose_a(*
transpose_b( 
�
Roptimizer/gradients_1/optimizer/main_graph_0/hidden_1/MatMul_grad/tuple/group_depsNoOpI^optimizer/gradients_1/optimizer/main_graph_0/hidden_1/MatMul_grad/MatMulK^optimizer/gradients_1/optimizer/main_graph_0/hidden_1/MatMul_grad/MatMul_1
�
Zoptimizer/gradients_1/optimizer/main_graph_0/hidden_1/MatMul_grad/tuple/control_dependencyIdentityHoptimizer/gradients_1/optimizer/main_graph_0/hidden_1/MatMul_grad/MatMulS^optimizer/gradients_1/optimizer/main_graph_0/hidden_1/MatMul_grad/tuple/group_deps*
T0*[
_classQ
OMloc:@optimizer/gradients_1/optimizer/main_graph_0/hidden_1/MatMul_grad/MatMul
�
\optimizer/gradients_1/optimizer/main_graph_0/hidden_1/MatMul_grad/tuple/control_dependency_1IdentityJoptimizer/gradients_1/optimizer/main_graph_0/hidden_1/MatMul_grad/MatMul_1S^optimizer/gradients_1/optimizer/main_graph_0/hidden_1/MatMul_grad/tuple/group_deps*
T0*]
_classS
QOloc:@optimizer/gradients_1/optimizer/main_graph_0/hidden_1/MatMul_grad/MatMul_1
c
/optimizer/gradients_1/policy_1/add_4_grad/ShapeShapepolicy_1/truediv*
T0*
out_type0
e
1optimizer/gradients_1/policy_1/add_4_grad/Shape_1Shapepolicy_1/add_4/y*
T0*
out_type0
�
?optimizer/gradients_1/policy_1/add_4_grad/BroadcastGradientArgsBroadcastGradientArgs/optimizer/gradients_1/policy_1/add_4_grad/Shape1optimizer/gradients_1/policy_1/add_4_grad/Shape_1*
T0
�
-optimizer/gradients_1/policy_1/add_4_grad/SumSum-optimizer/gradients_1/policy_1/Log_2_grad/mul?optimizer/gradients_1/policy_1/add_4_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
1optimizer/gradients_1/policy_1/add_4_grad/ReshapeReshape-optimizer/gradients_1/policy_1/add_4_grad/Sum/optimizer/gradients_1/policy_1/add_4_grad/Shape*
T0*
Tshape0
�
/optimizer/gradients_1/policy_1/add_4_grad/Sum_1Sum-optimizer/gradients_1/policy_1/Log_2_grad/mulAoptimizer/gradients_1/policy_1/add_4_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
3optimizer/gradients_1/policy_1/add_4_grad/Reshape_1Reshape/optimizer/gradients_1/policy_1/add_4_grad/Sum_11optimizer/gradients_1/policy_1/add_4_grad/Shape_1*
T0*
Tshape0
�
:optimizer/gradients_1/policy_1/add_4_grad/tuple/group_depsNoOp2^optimizer/gradients_1/policy_1/add_4_grad/Reshape4^optimizer/gradients_1/policy_1/add_4_grad/Reshape_1
�
Boptimizer/gradients_1/policy_1/add_4_grad/tuple/control_dependencyIdentity1optimizer/gradients_1/policy_1/add_4_grad/Reshape;^optimizer/gradients_1/policy_1/add_4_grad/tuple/group_deps*
T0*D
_class:
86loc:@optimizer/gradients_1/policy_1/add_4_grad/Reshape
�
Doptimizer/gradients_1/policy_1/add_4_grad/tuple/control_dependency_1Identity3optimizer/gradients_1/policy_1/add_4_grad/Reshape_1;^optimizer/gradients_1/policy_1/add_4_grad/tuple/group_deps*
T0*F
_class<
:8loc:@optimizer/gradients_1/policy_1/add_4_grad/Reshape_1
e
/optimizer/gradients_1/policy_1/add_5_grad/ShapeShapepolicy_1/truediv_1*
T0*
out_type0
e
1optimizer/gradients_1/policy_1/add_5_grad/Shape_1Shapepolicy_1/add_5/y*
T0*
out_type0
�
?optimizer/gradients_1/policy_1/add_5_grad/BroadcastGradientArgsBroadcastGradientArgs/optimizer/gradients_1/policy_1/add_5_grad/Shape1optimizer/gradients_1/policy_1/add_5_grad/Shape_1*
T0
�
-optimizer/gradients_1/policy_1/add_5_grad/SumSum-optimizer/gradients_1/policy_1/Log_3_grad/mul?optimizer/gradients_1/policy_1/add_5_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
1optimizer/gradients_1/policy_1/add_5_grad/ReshapeReshape-optimizer/gradients_1/policy_1/add_5_grad/Sum/optimizer/gradients_1/policy_1/add_5_grad/Shape*
T0*
Tshape0
�
/optimizer/gradients_1/policy_1/add_5_grad/Sum_1Sum-optimizer/gradients_1/policy_1/Log_3_grad/mulAoptimizer/gradients_1/policy_1/add_5_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
3optimizer/gradients_1/policy_1/add_5_grad/Reshape_1Reshape/optimizer/gradients_1/policy_1/add_5_grad/Sum_11optimizer/gradients_1/policy_1/add_5_grad/Shape_1*
T0*
Tshape0
�
:optimizer/gradients_1/policy_1/add_5_grad/tuple/group_depsNoOp2^optimizer/gradients_1/policy_1/add_5_grad/Reshape4^optimizer/gradients_1/policy_1/add_5_grad/Reshape_1
�
Boptimizer/gradients_1/policy_1/add_5_grad/tuple/control_dependencyIdentity1optimizer/gradients_1/policy_1/add_5_grad/Reshape;^optimizer/gradients_1/policy_1/add_5_grad/tuple/group_deps*
T0*D
_class:
86loc:@optimizer/gradients_1/policy_1/add_5_grad/Reshape
�
Doptimizer/gradients_1/policy_1/add_5_grad/tuple/control_dependency_1Identity3optimizer/gradients_1/policy_1/add_5_grad/Reshape_1;^optimizer/gradients_1/policy_1/add_5_grad/tuple/group_deps*
T0*F
_class<
:8loc:@optimizer/gradients_1/policy_1/add_5_grad/Reshape_1
�
Doptimizer/gradients_1/optimizer/main_graph_0/hidden_0/Mul_grad/ShapeShape'optimizer/main_graph_0/hidden_0/BiasAdd*
T0*
out_type0
�
Foptimizer/gradients_1/optimizer/main_graph_0/hidden_0/Mul_grad/Shape_1Shape'optimizer/main_graph_0/hidden_0/Sigmoid*
T0*
out_type0
�
Toptimizer/gradients_1/optimizer/main_graph_0/hidden_0/Mul_grad/BroadcastGradientArgsBroadcastGradientArgsDoptimizer/gradients_1/optimizer/main_graph_0/hidden_0/Mul_grad/ShapeFoptimizer/gradients_1/optimizer/main_graph_0/hidden_0/Mul_grad/Shape_1*
T0
�
Boptimizer/gradients_1/optimizer/main_graph_0/hidden_0/Mul_grad/MulMulZoptimizer/gradients_1/optimizer/main_graph_0/hidden_1/MatMul_grad/tuple/control_dependency'optimizer/main_graph_0/hidden_0/Sigmoid*
T0
�
Boptimizer/gradients_1/optimizer/main_graph_0/hidden_0/Mul_grad/SumSumBoptimizer/gradients_1/optimizer/main_graph_0/hidden_0/Mul_grad/MulToptimizer/gradients_1/optimizer/main_graph_0/hidden_0/Mul_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
Foptimizer/gradients_1/optimizer/main_graph_0/hidden_0/Mul_grad/ReshapeReshapeBoptimizer/gradients_1/optimizer/main_graph_0/hidden_0/Mul_grad/SumDoptimizer/gradients_1/optimizer/main_graph_0/hidden_0/Mul_grad/Shape*
T0*
Tshape0
�
Doptimizer/gradients_1/optimizer/main_graph_0/hidden_0/Mul_grad/Mul_1Mul'optimizer/main_graph_0/hidden_0/BiasAddZoptimizer/gradients_1/optimizer/main_graph_0/hidden_1/MatMul_grad/tuple/control_dependency*
T0
�
Doptimizer/gradients_1/optimizer/main_graph_0/hidden_0/Mul_grad/Sum_1SumDoptimizer/gradients_1/optimizer/main_graph_0/hidden_0/Mul_grad/Mul_1Voptimizer/gradients_1/optimizer/main_graph_0/hidden_0/Mul_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
Hoptimizer/gradients_1/optimizer/main_graph_0/hidden_0/Mul_grad/Reshape_1ReshapeDoptimizer/gradients_1/optimizer/main_graph_0/hidden_0/Mul_grad/Sum_1Foptimizer/gradients_1/optimizer/main_graph_0/hidden_0/Mul_grad/Shape_1*
T0*
Tshape0
�
Ooptimizer/gradients_1/optimizer/main_graph_0/hidden_0/Mul_grad/tuple/group_depsNoOpG^optimizer/gradients_1/optimizer/main_graph_0/hidden_0/Mul_grad/ReshapeI^optimizer/gradients_1/optimizer/main_graph_0/hidden_0/Mul_grad/Reshape_1
�
Woptimizer/gradients_1/optimizer/main_graph_0/hidden_0/Mul_grad/tuple/control_dependencyIdentityFoptimizer/gradients_1/optimizer/main_graph_0/hidden_0/Mul_grad/ReshapeP^optimizer/gradients_1/optimizer/main_graph_0/hidden_0/Mul_grad/tuple/group_deps*
T0*Y
_classO
MKloc:@optimizer/gradients_1/optimizer/main_graph_0/hidden_0/Mul_grad/Reshape
�
Yoptimizer/gradients_1/optimizer/main_graph_0/hidden_0/Mul_grad/tuple/control_dependency_1IdentityHoptimizer/gradients_1/optimizer/main_graph_0/hidden_0/Mul_grad/Reshape_1P^optimizer/gradients_1/optimizer/main_graph_0/hidden_0/Mul_grad/tuple/group_deps*
T0*[
_classQ
OMloc:@optimizer/gradients_1/optimizer/main_graph_0/hidden_0/Mul_grad/Reshape_1
a
1optimizer/gradients_1/policy_1/truediv_grad/ShapeShapepolicy_1/Mul*
T0*
out_type0
c
3optimizer/gradients_1/policy_1/truediv_grad/Shape_1Shapepolicy_1/Sum*
T0*
out_type0
�
Aoptimizer/gradients_1/policy_1/truediv_grad/BroadcastGradientArgsBroadcastGradientArgs1optimizer/gradients_1/policy_1/truediv_grad/Shape3optimizer/gradients_1/policy_1/truediv_grad/Shape_1*
T0
�
3optimizer/gradients_1/policy_1/truediv_grad/RealDivRealDivBoptimizer/gradients_1/policy_1/add_4_grad/tuple/control_dependencypolicy_1/Sum*
T0
�
/optimizer/gradients_1/policy_1/truediv_grad/SumSum3optimizer/gradients_1/policy_1/truediv_grad/RealDivAoptimizer/gradients_1/policy_1/truediv_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
3optimizer/gradients_1/policy_1/truediv_grad/ReshapeReshape/optimizer/gradients_1/policy_1/truediv_grad/Sum1optimizer/gradients_1/policy_1/truediv_grad/Shape*
T0*
Tshape0
M
/optimizer/gradients_1/policy_1/truediv_grad/NegNegpolicy_1/Mul*
T0
�
5optimizer/gradients_1/policy_1/truediv_grad/RealDiv_1RealDiv/optimizer/gradients_1/policy_1/truediv_grad/Negpolicy_1/Sum*
T0
�
5optimizer/gradients_1/policy_1/truediv_grad/RealDiv_2RealDiv5optimizer/gradients_1/policy_1/truediv_grad/RealDiv_1policy_1/Sum*
T0
�
/optimizer/gradients_1/policy_1/truediv_grad/mulMulBoptimizer/gradients_1/policy_1/add_4_grad/tuple/control_dependency5optimizer/gradients_1/policy_1/truediv_grad/RealDiv_2*
T0
�
1optimizer/gradients_1/policy_1/truediv_grad/Sum_1Sum/optimizer/gradients_1/policy_1/truediv_grad/mulCoptimizer/gradients_1/policy_1/truediv_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
5optimizer/gradients_1/policy_1/truediv_grad/Reshape_1Reshape1optimizer/gradients_1/policy_1/truediv_grad/Sum_13optimizer/gradients_1/policy_1/truediv_grad/Shape_1*
T0*
Tshape0
�
<optimizer/gradients_1/policy_1/truediv_grad/tuple/group_depsNoOp4^optimizer/gradients_1/policy_1/truediv_grad/Reshape6^optimizer/gradients_1/policy_1/truediv_grad/Reshape_1
�
Doptimizer/gradients_1/policy_1/truediv_grad/tuple/control_dependencyIdentity3optimizer/gradients_1/policy_1/truediv_grad/Reshape=^optimizer/gradients_1/policy_1/truediv_grad/tuple/group_deps*
T0*F
_class<
:8loc:@optimizer/gradients_1/policy_1/truediv_grad/Reshape
�
Foptimizer/gradients_1/policy_1/truediv_grad/tuple/control_dependency_1Identity5optimizer/gradients_1/policy_1/truediv_grad/Reshape_1=^optimizer/gradients_1/policy_1/truediv_grad/tuple/group_deps*
T0*H
_class>
<:loc:@optimizer/gradients_1/policy_1/truediv_grad/Reshape_1
e
3optimizer/gradients_1/policy_1/truediv_1_grad/ShapeShapepolicy_1/Mul_1*
T0*
out_type0
g
5optimizer/gradients_1/policy_1/truediv_1_grad/Shape_1Shapepolicy_1/Sum_1*
T0*
out_type0
�
Coptimizer/gradients_1/policy_1/truediv_1_grad/BroadcastGradientArgsBroadcastGradientArgs3optimizer/gradients_1/policy_1/truediv_1_grad/Shape5optimizer/gradients_1/policy_1/truediv_1_grad/Shape_1*
T0
�
5optimizer/gradients_1/policy_1/truediv_1_grad/RealDivRealDivBoptimizer/gradients_1/policy_1/add_5_grad/tuple/control_dependencypolicy_1/Sum_1*
T0
�
1optimizer/gradients_1/policy_1/truediv_1_grad/SumSum5optimizer/gradients_1/policy_1/truediv_1_grad/RealDivCoptimizer/gradients_1/policy_1/truediv_1_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
5optimizer/gradients_1/policy_1/truediv_1_grad/ReshapeReshape1optimizer/gradients_1/policy_1/truediv_1_grad/Sum3optimizer/gradients_1/policy_1/truediv_1_grad/Shape*
T0*
Tshape0
Q
1optimizer/gradients_1/policy_1/truediv_1_grad/NegNegpolicy_1/Mul_1*
T0
�
7optimizer/gradients_1/policy_1/truediv_1_grad/RealDiv_1RealDiv1optimizer/gradients_1/policy_1/truediv_1_grad/Negpolicy_1/Sum_1*
T0
�
7optimizer/gradients_1/policy_1/truediv_1_grad/RealDiv_2RealDiv7optimizer/gradients_1/policy_1/truediv_1_grad/RealDiv_1policy_1/Sum_1*
T0
�
1optimizer/gradients_1/policy_1/truediv_1_grad/mulMulBoptimizer/gradients_1/policy_1/add_5_grad/tuple/control_dependency7optimizer/gradients_1/policy_1/truediv_1_grad/RealDiv_2*
T0
�
3optimizer/gradients_1/policy_1/truediv_1_grad/Sum_1Sum1optimizer/gradients_1/policy_1/truediv_1_grad/mulEoptimizer/gradients_1/policy_1/truediv_1_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
7optimizer/gradients_1/policy_1/truediv_1_grad/Reshape_1Reshape3optimizer/gradients_1/policy_1/truediv_1_grad/Sum_15optimizer/gradients_1/policy_1/truediv_1_grad/Shape_1*
T0*
Tshape0
�
>optimizer/gradients_1/policy_1/truediv_1_grad/tuple/group_depsNoOp6^optimizer/gradients_1/policy_1/truediv_1_grad/Reshape8^optimizer/gradients_1/policy_1/truediv_1_grad/Reshape_1
�
Foptimizer/gradients_1/policy_1/truediv_1_grad/tuple/control_dependencyIdentity5optimizer/gradients_1/policy_1/truediv_1_grad/Reshape?^optimizer/gradients_1/policy_1/truediv_1_grad/tuple/group_deps*
T0*H
_class>
<:loc:@optimizer/gradients_1/policy_1/truediv_1_grad/Reshape
�
Hoptimizer/gradients_1/policy_1/truediv_1_grad/tuple/control_dependency_1Identity7optimizer/gradients_1/policy_1/truediv_1_grad/Reshape_1?^optimizer/gradients_1/policy_1/truediv_1_grad/tuple/group_deps*
T0*J
_class@
><loc:@optimizer/gradients_1/policy_1/truediv_1_grad/Reshape_1
�
Noptimizer/gradients_1/optimizer/main_graph_0/hidden_0/Sigmoid_grad/SigmoidGradSigmoidGrad'optimizer/main_graph_0/hidden_0/SigmoidYoptimizer/gradients_1/optimizer/main_graph_0/hidden_0/Mul_grad/tuple/control_dependency_1*
T0
]
-optimizer/gradients_1/policy_1/Sum_grad/ShapeShapepolicy_1/Mul*
T0*
out_type0
�
3optimizer/gradients_1/policy_1/Sum_grad/BroadcastToBroadcastToFoptimizer/gradients_1/policy_1/truediv_grad/tuple/control_dependency_1-optimizer/gradients_1/policy_1/Sum_grad/Shape*
T0*

Tidx0
a
/optimizer/gradients_1/policy_1/Sum_1_grad/ShapeShapepolicy_1/Mul_1*
T0*
out_type0
�
5optimizer/gradients_1/policy_1/Sum_1_grad/BroadcastToBroadcastToHoptimizer/gradients_1/policy_1/truediv_1_grad/tuple/control_dependency_1/optimizer/gradients_1/policy_1/Sum_1_grad/Shape*
T0*

Tidx0
�
optimizer/gradients_1/AddN_6AddNWoptimizer/gradients_1/optimizer/main_graph_0/hidden_0/Mul_grad/tuple/control_dependencyNoptimizer/gradients_1/optimizer/main_graph_0/hidden_0/Sigmoid_grad/SigmoidGrad*
N*
T0*Y
_classO
MKloc:@optimizer/gradients_1/optimizer/main_graph_0/hidden_0/Mul_grad/Reshape
�
Noptimizer/gradients_1/optimizer/main_graph_0/hidden_0/BiasAdd_grad/BiasAddGradBiasAddGradoptimizer/gradients_1/AddN_6*
T0*
data_formatNHWC
�
Soptimizer/gradients_1/optimizer/main_graph_0/hidden_0/BiasAdd_grad/tuple/group_depsNoOp^optimizer/gradients_1/AddN_6O^optimizer/gradients_1/optimizer/main_graph_0/hidden_0/BiasAdd_grad/BiasAddGrad
�
[optimizer/gradients_1/optimizer/main_graph_0/hidden_0/BiasAdd_grad/tuple/control_dependencyIdentityoptimizer/gradients_1/AddN_6T^optimizer/gradients_1/optimizer/main_graph_0/hidden_0/BiasAdd_grad/tuple/group_deps*
T0*Y
_classO
MKloc:@optimizer/gradients_1/optimizer/main_graph_0/hidden_0/Mul_grad/Reshape
�
]optimizer/gradients_1/optimizer/main_graph_0/hidden_0/BiasAdd_grad/tuple/control_dependency_1IdentityNoptimizer/gradients_1/optimizer/main_graph_0/hidden_0/BiasAdd_grad/BiasAddGradT^optimizer/gradients_1/optimizer/main_graph_0/hidden_0/BiasAdd_grad/tuple/group_deps*
T0*a
_classW
USloc:@optimizer/gradients_1/optimizer/main_graph_0/hidden_0/BiasAdd_grad/BiasAddGrad
�
optimizer/gradients_1/AddN_7AddNDoptimizer/gradients_1/policy_1/truediv_grad/tuple/control_dependency3optimizer/gradients_1/policy_1/Sum_grad/BroadcastTo*
N*
T0*F
_class<
:8loc:@optimizer/gradients_1/policy_1/truediv_grad/Reshape
]
-optimizer/gradients_1/policy_1/Mul_grad/ShapeShapepolicy_1/add*
T0*
out_type0
i
/optimizer/gradients_1/policy_1/Mul_grad/Shape_1Shapepolicy_1/strided_slice*
T0*
out_type0
�
=optimizer/gradients_1/policy_1/Mul_grad/BroadcastGradientArgsBroadcastGradientArgs-optimizer/gradients_1/policy_1/Mul_grad/Shape/optimizer/gradients_1/policy_1/Mul_grad/Shape_1*
T0
q
+optimizer/gradients_1/policy_1/Mul_grad/MulMuloptimizer/gradients_1/AddN_7policy_1/strided_slice*
T0
�
+optimizer/gradients_1/policy_1/Mul_grad/SumSum+optimizer/gradients_1/policy_1/Mul_grad/Mul=optimizer/gradients_1/policy_1/Mul_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
/optimizer/gradients_1/policy_1/Mul_grad/ReshapeReshape+optimizer/gradients_1/policy_1/Mul_grad/Sum-optimizer/gradients_1/policy_1/Mul_grad/Shape*
T0*
Tshape0
i
-optimizer/gradients_1/policy_1/Mul_grad/Mul_1Mulpolicy_1/addoptimizer/gradients_1/AddN_7*
T0
�
-optimizer/gradients_1/policy_1/Mul_grad/Sum_1Sum-optimizer/gradients_1/policy_1/Mul_grad/Mul_1?optimizer/gradients_1/policy_1/Mul_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
1optimizer/gradients_1/policy_1/Mul_grad/Reshape_1Reshape-optimizer/gradients_1/policy_1/Mul_grad/Sum_1/optimizer/gradients_1/policy_1/Mul_grad/Shape_1*
T0*
Tshape0
�
8optimizer/gradients_1/policy_1/Mul_grad/tuple/group_depsNoOp0^optimizer/gradients_1/policy_1/Mul_grad/Reshape2^optimizer/gradients_1/policy_1/Mul_grad/Reshape_1
�
@optimizer/gradients_1/policy_1/Mul_grad/tuple/control_dependencyIdentity/optimizer/gradients_1/policy_1/Mul_grad/Reshape9^optimizer/gradients_1/policy_1/Mul_grad/tuple/group_deps*
T0*B
_class8
64loc:@optimizer/gradients_1/policy_1/Mul_grad/Reshape
�
Boptimizer/gradients_1/policy_1/Mul_grad/tuple/control_dependency_1Identity1optimizer/gradients_1/policy_1/Mul_grad/Reshape_19^optimizer/gradients_1/policy_1/Mul_grad/tuple/group_deps*
T0*D
_class:
86loc:@optimizer/gradients_1/policy_1/Mul_grad/Reshape_1
�
optimizer/gradients_1/AddN_8AddNFoptimizer/gradients_1/policy_1/truediv_1_grad/tuple/control_dependency5optimizer/gradients_1/policy_1/Sum_1_grad/BroadcastTo*
N*
T0*H
_class>
<:loc:@optimizer/gradients_1/policy_1/truediv_1_grad/Reshape
a
/optimizer/gradients_1/policy_1/Mul_1_grad/ShapeShapepolicy_1/add_1*
T0*
out_type0
m
1optimizer/gradients_1/policy_1/Mul_1_grad/Shape_1Shapepolicy_1/strided_slice_1*
T0*
out_type0
�
?optimizer/gradients_1/policy_1/Mul_1_grad/BroadcastGradientArgsBroadcastGradientArgs/optimizer/gradients_1/policy_1/Mul_1_grad/Shape1optimizer/gradients_1/policy_1/Mul_1_grad/Shape_1*
T0
u
-optimizer/gradients_1/policy_1/Mul_1_grad/MulMuloptimizer/gradients_1/AddN_8policy_1/strided_slice_1*
T0
�
-optimizer/gradients_1/policy_1/Mul_1_grad/SumSum-optimizer/gradients_1/policy_1/Mul_1_grad/Mul?optimizer/gradients_1/policy_1/Mul_1_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
1optimizer/gradients_1/policy_1/Mul_1_grad/ReshapeReshape-optimizer/gradients_1/policy_1/Mul_1_grad/Sum/optimizer/gradients_1/policy_1/Mul_1_grad/Shape*
T0*
Tshape0
m
/optimizer/gradients_1/policy_1/Mul_1_grad/Mul_1Mulpolicy_1/add_1optimizer/gradients_1/AddN_8*
T0
�
/optimizer/gradients_1/policy_1/Mul_1_grad/Sum_1Sum/optimizer/gradients_1/policy_1/Mul_1_grad/Mul_1Aoptimizer/gradients_1/policy_1/Mul_1_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
3optimizer/gradients_1/policy_1/Mul_1_grad/Reshape_1Reshape/optimizer/gradients_1/policy_1/Mul_1_grad/Sum_11optimizer/gradients_1/policy_1/Mul_1_grad/Shape_1*
T0*
Tshape0
�
:optimizer/gradients_1/policy_1/Mul_1_grad/tuple/group_depsNoOp2^optimizer/gradients_1/policy_1/Mul_1_grad/Reshape4^optimizer/gradients_1/policy_1/Mul_1_grad/Reshape_1
�
Boptimizer/gradients_1/policy_1/Mul_1_grad/tuple/control_dependencyIdentity1optimizer/gradients_1/policy_1/Mul_1_grad/Reshape;^optimizer/gradients_1/policy_1/Mul_1_grad/tuple/group_deps*
T0*D
_class:
86loc:@optimizer/gradients_1/policy_1/Mul_1_grad/Reshape
�
Doptimizer/gradients_1/policy_1/Mul_1_grad/tuple/control_dependency_1Identity3optimizer/gradients_1/policy_1/Mul_1_grad/Reshape_1;^optimizer/gradients_1/policy_1/Mul_1_grad/tuple/group_deps*
T0*F
_class<
:8loc:@optimizer/gradients_1/policy_1/Mul_1_grad/Reshape_1
�
Hoptimizer/gradients_1/optimizer/main_graph_0/hidden_0/MatMul_grad/MatMulMatMul[optimizer/gradients_1/optimizer/main_graph_0/hidden_0/BiasAdd_grad/tuple/control_dependency,optimizer//main_graph_0/hidden_0/kernel/read*
T0*
transpose_a( *
transpose_b(
�
Joptimizer/gradients_1/optimizer/main_graph_0/hidden_0/MatMul_grad/MatMul_1MatMulvector_observation[optimizer/gradients_1/optimizer/main_graph_0/hidden_0/BiasAdd_grad/tuple/control_dependency*
T0*
transpose_a(*
transpose_b( 
�
Roptimizer/gradients_1/optimizer/main_graph_0/hidden_0/MatMul_grad/tuple/group_depsNoOpI^optimizer/gradients_1/optimizer/main_graph_0/hidden_0/MatMul_grad/MatMulK^optimizer/gradients_1/optimizer/main_graph_0/hidden_0/MatMul_grad/MatMul_1
�
Zoptimizer/gradients_1/optimizer/main_graph_0/hidden_0/MatMul_grad/tuple/control_dependencyIdentityHoptimizer/gradients_1/optimizer/main_graph_0/hidden_0/MatMul_grad/MatMulS^optimizer/gradients_1/optimizer/main_graph_0/hidden_0/MatMul_grad/tuple/group_deps*
T0*[
_classQ
OMloc:@optimizer/gradients_1/optimizer/main_graph_0/hidden_0/MatMul_grad/MatMul
�
\optimizer/gradients_1/optimizer/main_graph_0/hidden_0/MatMul_grad/tuple/control_dependency_1IdentityJoptimizer/gradients_1/optimizer/main_graph_0/hidden_0/MatMul_grad/MatMul_1S^optimizer/gradients_1/optimizer/main_graph_0/hidden_0/MatMul_grad/tuple/group_deps*
T0*]
_classS
QOloc:@optimizer/gradients_1/optimizer/main_graph_0/hidden_0/MatMul_grad/MatMul_1
a
-optimizer/gradients_1/policy_1/add_grad/ShapeShapepolicy_1/Softmax*
T0*
out_type0
a
/optimizer/gradients_1/policy_1/add_grad/Shape_1Shapepolicy_1/add/y*
T0*
out_type0
�
=optimizer/gradients_1/policy_1/add_grad/BroadcastGradientArgsBroadcastGradientArgs-optimizer/gradients_1/policy_1/add_grad/Shape/optimizer/gradients_1/policy_1/add_grad/Shape_1*
T0
�
+optimizer/gradients_1/policy_1/add_grad/SumSum@optimizer/gradients_1/policy_1/Mul_grad/tuple/control_dependency=optimizer/gradients_1/policy_1/add_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
/optimizer/gradients_1/policy_1/add_grad/ReshapeReshape+optimizer/gradients_1/policy_1/add_grad/Sum-optimizer/gradients_1/policy_1/add_grad/Shape*
T0*
Tshape0
�
-optimizer/gradients_1/policy_1/add_grad/Sum_1Sum@optimizer/gradients_1/policy_1/Mul_grad/tuple/control_dependency?optimizer/gradients_1/policy_1/add_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
1optimizer/gradients_1/policy_1/add_grad/Reshape_1Reshape-optimizer/gradients_1/policy_1/add_grad/Sum_1/optimizer/gradients_1/policy_1/add_grad/Shape_1*
T0*
Tshape0
�
8optimizer/gradients_1/policy_1/add_grad/tuple/group_depsNoOp0^optimizer/gradients_1/policy_1/add_grad/Reshape2^optimizer/gradients_1/policy_1/add_grad/Reshape_1
�
@optimizer/gradients_1/policy_1/add_grad/tuple/control_dependencyIdentity/optimizer/gradients_1/policy_1/add_grad/Reshape9^optimizer/gradients_1/policy_1/add_grad/tuple/group_deps*
T0*B
_class8
64loc:@optimizer/gradients_1/policy_1/add_grad/Reshape
�
Boptimizer/gradients_1/policy_1/add_grad/tuple/control_dependency_1Identity1optimizer/gradients_1/policy_1/add_grad/Reshape_19^optimizer/gradients_1/policy_1/add_grad/tuple/group_deps*
T0*D
_class:
86loc:@optimizer/gradients_1/policy_1/add_grad/Reshape_1
e
/optimizer/gradients_1/policy_1/add_1_grad/ShapeShapepolicy_1/Softmax_1*
T0*
out_type0
e
1optimizer/gradients_1/policy_1/add_1_grad/Shape_1Shapepolicy_1/add_1/y*
T0*
out_type0
�
?optimizer/gradients_1/policy_1/add_1_grad/BroadcastGradientArgsBroadcastGradientArgs/optimizer/gradients_1/policy_1/add_1_grad/Shape1optimizer/gradients_1/policy_1/add_1_grad/Shape_1*
T0
�
-optimizer/gradients_1/policy_1/add_1_grad/SumSumBoptimizer/gradients_1/policy_1/Mul_1_grad/tuple/control_dependency?optimizer/gradients_1/policy_1/add_1_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
1optimizer/gradients_1/policy_1/add_1_grad/ReshapeReshape-optimizer/gradients_1/policy_1/add_1_grad/Sum/optimizer/gradients_1/policy_1/add_1_grad/Shape*
T0*
Tshape0
�
/optimizer/gradients_1/policy_1/add_1_grad/Sum_1SumBoptimizer/gradients_1/policy_1/Mul_1_grad/tuple/control_dependencyAoptimizer/gradients_1/policy_1/add_1_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
3optimizer/gradients_1/policy_1/add_1_grad/Reshape_1Reshape/optimizer/gradients_1/policy_1/add_1_grad/Sum_11optimizer/gradients_1/policy_1/add_1_grad/Shape_1*
T0*
Tshape0
�
:optimizer/gradients_1/policy_1/add_1_grad/tuple/group_depsNoOp2^optimizer/gradients_1/policy_1/add_1_grad/Reshape4^optimizer/gradients_1/policy_1/add_1_grad/Reshape_1
�
Boptimizer/gradients_1/policy_1/add_1_grad/tuple/control_dependencyIdentity1optimizer/gradients_1/policy_1/add_1_grad/Reshape;^optimizer/gradients_1/policy_1/add_1_grad/tuple/group_deps*
T0*D
_class:
86loc:@optimizer/gradients_1/policy_1/add_1_grad/Reshape
�
Doptimizer/gradients_1/policy_1/add_1_grad/tuple/control_dependency_1Identity3optimizer/gradients_1/policy_1/add_1_grad/Reshape_1;^optimizer/gradients_1/policy_1/add_1_grad/tuple/group_deps*
T0*F
_class<
:8loc:@optimizer/gradients_1/policy_1/add_1_grad/Reshape_1
�
/optimizer/gradients_1/policy_1/Softmax_grad/mulMul@optimizer/gradients_1/policy_1/add_grad/tuple/control_dependencypolicy_1/Softmax*
T0
t
Aoptimizer/gradients_1/policy_1/Softmax_grad/Sum/reduction_indicesConst*
dtype0*
valueB :
���������
�
/optimizer/gradients_1/policy_1/Softmax_grad/SumSum/optimizer/gradients_1/policy_1/Softmax_grad/mulAoptimizer/gradients_1/policy_1/Softmax_grad/Sum/reduction_indices*
T0*

Tidx0*
	keep_dims(
�
/optimizer/gradients_1/policy_1/Softmax_grad/subSub@optimizer/gradients_1/policy_1/add_grad/tuple/control_dependency/optimizer/gradients_1/policy_1/Softmax_grad/Sum*
T0
�
1optimizer/gradients_1/policy_1/Softmax_grad/mul_1Mul/optimizer/gradients_1/policy_1/Softmax_grad/subpolicy_1/Softmax*
T0
�
1optimizer/gradients_1/policy_1/Softmax_1_grad/mulMulBoptimizer/gradients_1/policy_1/add_1_grad/tuple/control_dependencypolicy_1/Softmax_1*
T0
v
Coptimizer/gradients_1/policy_1/Softmax_1_grad/Sum/reduction_indicesConst*
dtype0*
valueB :
���������
�
1optimizer/gradients_1/policy_1/Softmax_1_grad/SumSum1optimizer/gradients_1/policy_1/Softmax_1_grad/mulCoptimizer/gradients_1/policy_1/Softmax_1_grad/Sum/reduction_indices*
T0*

Tidx0*
	keep_dims(
�
1optimizer/gradients_1/policy_1/Softmax_1_grad/subSubBoptimizer/gradients_1/policy_1/add_1_grad/tuple/control_dependency1optimizer/gradients_1/policy_1/Softmax_1_grad/Sum*
T0
�
3optimizer/gradients_1/policy_1/Softmax_1_grad/mul_1Mul1optimizer/gradients_1/policy_1/Softmax_1_grad/subpolicy_1/Softmax_1*
T0
�
7optimizer/gradients_1/policy_1/dense/MatMul_grad/MatMulMatMul1optimizer/gradients_1/policy_1/Softmax_grad/mul_1policy/dense/kernel/read*
T0*
transpose_a( *
transpose_b(
�
9optimizer/gradients_1/policy_1/dense/MatMul_grad/MatMul_1MatMul policy/main_graph_0/hidden_1/Mul1optimizer/gradients_1/policy_1/Softmax_grad/mul_1*
T0*
transpose_a(*
transpose_b( 
�
Aoptimizer/gradients_1/policy_1/dense/MatMul_grad/tuple/group_depsNoOp8^optimizer/gradients_1/policy_1/dense/MatMul_grad/MatMul:^optimizer/gradients_1/policy_1/dense/MatMul_grad/MatMul_1
�
Ioptimizer/gradients_1/policy_1/dense/MatMul_grad/tuple/control_dependencyIdentity7optimizer/gradients_1/policy_1/dense/MatMul_grad/MatMulB^optimizer/gradients_1/policy_1/dense/MatMul_grad/tuple/group_deps*
T0*J
_class@
><loc:@optimizer/gradients_1/policy_1/dense/MatMul_grad/MatMul
�
Koptimizer/gradients_1/policy_1/dense/MatMul_grad/tuple/control_dependency_1Identity9optimizer/gradients_1/policy_1/dense/MatMul_grad/MatMul_1B^optimizer/gradients_1/policy_1/dense/MatMul_grad/tuple/group_deps*
T0*L
_classB
@>loc:@optimizer/gradients_1/policy_1/dense/MatMul_grad/MatMul_1
�
9optimizer/gradients_1/policy_1/dense_1/MatMul_grad/MatMulMatMul3optimizer/gradients_1/policy_1/Softmax_1_grad/mul_1policy/dense_1/kernel/read*
T0*
transpose_a( *
transpose_b(
�
;optimizer/gradients_1/policy_1/dense_1/MatMul_grad/MatMul_1MatMul policy/main_graph_0/hidden_1/Mul3optimizer/gradients_1/policy_1/Softmax_1_grad/mul_1*
T0*
transpose_a(*
transpose_b( 
�
Coptimizer/gradients_1/policy_1/dense_1/MatMul_grad/tuple/group_depsNoOp:^optimizer/gradients_1/policy_1/dense_1/MatMul_grad/MatMul<^optimizer/gradients_1/policy_1/dense_1/MatMul_grad/MatMul_1
�
Koptimizer/gradients_1/policy_1/dense_1/MatMul_grad/tuple/control_dependencyIdentity9optimizer/gradients_1/policy_1/dense_1/MatMul_grad/MatMulD^optimizer/gradients_1/policy_1/dense_1/MatMul_grad/tuple/group_deps*
T0*L
_classB
@>loc:@optimizer/gradients_1/policy_1/dense_1/MatMul_grad/MatMul
�
Moptimizer/gradients_1/policy_1/dense_1/MatMul_grad/tuple/control_dependency_1Identity;optimizer/gradients_1/policy_1/dense_1/MatMul_grad/MatMul_1D^optimizer/gradients_1/policy_1/dense_1/MatMul_grad/tuple/group_deps*
T0*N
_classD
B@loc:@optimizer/gradients_1/policy_1/dense_1/MatMul_grad/MatMul_1
�
optimizer/gradients_1/AddN_9AddNIoptimizer/gradients_1/policy_1/dense/MatMul_grad/tuple/control_dependencyKoptimizer/gradients_1/policy_1/dense_1/MatMul_grad/tuple/control_dependency*
N*
T0*J
_class@
><loc:@optimizer/gradients_1/policy_1/dense/MatMul_grad/MatMul
�
Aoptimizer/gradients_1/policy/main_graph_0/hidden_1/Mul_grad/ShapeShape$policy/main_graph_0/hidden_1/BiasAdd*
T0*
out_type0
�
Coptimizer/gradients_1/policy/main_graph_0/hidden_1/Mul_grad/Shape_1Shape$policy/main_graph_0/hidden_1/Sigmoid*
T0*
out_type0
�
Qoptimizer/gradients_1/policy/main_graph_0/hidden_1/Mul_grad/BroadcastGradientArgsBroadcastGradientArgsAoptimizer/gradients_1/policy/main_graph_0/hidden_1/Mul_grad/ShapeCoptimizer/gradients_1/policy/main_graph_0/hidden_1/Mul_grad/Shape_1*
T0
�
?optimizer/gradients_1/policy/main_graph_0/hidden_1/Mul_grad/MulMuloptimizer/gradients_1/AddN_9$policy/main_graph_0/hidden_1/Sigmoid*
T0
�
?optimizer/gradients_1/policy/main_graph_0/hidden_1/Mul_grad/SumSum?optimizer/gradients_1/policy/main_graph_0/hidden_1/Mul_grad/MulQoptimizer/gradients_1/policy/main_graph_0/hidden_1/Mul_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
Coptimizer/gradients_1/policy/main_graph_0/hidden_1/Mul_grad/ReshapeReshape?optimizer/gradients_1/policy/main_graph_0/hidden_1/Mul_grad/SumAoptimizer/gradients_1/policy/main_graph_0/hidden_1/Mul_grad/Shape*
T0*
Tshape0
�
Aoptimizer/gradients_1/policy/main_graph_0/hidden_1/Mul_grad/Mul_1Mul$policy/main_graph_0/hidden_1/BiasAddoptimizer/gradients_1/AddN_9*
T0
�
Aoptimizer/gradients_1/policy/main_graph_0/hidden_1/Mul_grad/Sum_1SumAoptimizer/gradients_1/policy/main_graph_0/hidden_1/Mul_grad/Mul_1Soptimizer/gradients_1/policy/main_graph_0/hidden_1/Mul_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
Eoptimizer/gradients_1/policy/main_graph_0/hidden_1/Mul_grad/Reshape_1ReshapeAoptimizer/gradients_1/policy/main_graph_0/hidden_1/Mul_grad/Sum_1Coptimizer/gradients_1/policy/main_graph_0/hidden_1/Mul_grad/Shape_1*
T0*
Tshape0
�
Loptimizer/gradients_1/policy/main_graph_0/hidden_1/Mul_grad/tuple/group_depsNoOpD^optimizer/gradients_1/policy/main_graph_0/hidden_1/Mul_grad/ReshapeF^optimizer/gradients_1/policy/main_graph_0/hidden_1/Mul_grad/Reshape_1
�
Toptimizer/gradients_1/policy/main_graph_0/hidden_1/Mul_grad/tuple/control_dependencyIdentityCoptimizer/gradients_1/policy/main_graph_0/hidden_1/Mul_grad/ReshapeM^optimizer/gradients_1/policy/main_graph_0/hidden_1/Mul_grad/tuple/group_deps*
T0*V
_classL
JHloc:@optimizer/gradients_1/policy/main_graph_0/hidden_1/Mul_grad/Reshape
�
Voptimizer/gradients_1/policy/main_graph_0/hidden_1/Mul_grad/tuple/control_dependency_1IdentityEoptimizer/gradients_1/policy/main_graph_0/hidden_1/Mul_grad/Reshape_1M^optimizer/gradients_1/policy/main_graph_0/hidden_1/Mul_grad/tuple/group_deps*
T0*X
_classN
LJloc:@optimizer/gradients_1/policy/main_graph_0/hidden_1/Mul_grad/Reshape_1
�
Koptimizer/gradients_1/policy/main_graph_0/hidden_1/Sigmoid_grad/SigmoidGradSigmoidGrad$policy/main_graph_0/hidden_1/SigmoidVoptimizer/gradients_1/policy/main_graph_0/hidden_1/Mul_grad/tuple/control_dependency_1*
T0
�
optimizer/gradients_1/AddN_10AddNToptimizer/gradients_1/policy/main_graph_0/hidden_1/Mul_grad/tuple/control_dependencyKoptimizer/gradients_1/policy/main_graph_0/hidden_1/Sigmoid_grad/SigmoidGrad*
N*
T0*V
_classL
JHloc:@optimizer/gradients_1/policy/main_graph_0/hidden_1/Mul_grad/Reshape
�
Koptimizer/gradients_1/policy/main_graph_0/hidden_1/BiasAdd_grad/BiasAddGradBiasAddGradoptimizer/gradients_1/AddN_10*
T0*
data_formatNHWC
�
Poptimizer/gradients_1/policy/main_graph_0/hidden_1/BiasAdd_grad/tuple/group_depsNoOp^optimizer/gradients_1/AddN_10L^optimizer/gradients_1/policy/main_graph_0/hidden_1/BiasAdd_grad/BiasAddGrad
�
Xoptimizer/gradients_1/policy/main_graph_0/hidden_1/BiasAdd_grad/tuple/control_dependencyIdentityoptimizer/gradients_1/AddN_10Q^optimizer/gradients_1/policy/main_graph_0/hidden_1/BiasAdd_grad/tuple/group_deps*
T0*V
_classL
JHloc:@optimizer/gradients_1/policy/main_graph_0/hidden_1/Mul_grad/Reshape
�
Zoptimizer/gradients_1/policy/main_graph_0/hidden_1/BiasAdd_grad/tuple/control_dependency_1IdentityKoptimizer/gradients_1/policy/main_graph_0/hidden_1/BiasAdd_grad/BiasAddGradQ^optimizer/gradients_1/policy/main_graph_0/hidden_1/BiasAdd_grad/tuple/group_deps*
T0*^
_classT
RPloc:@optimizer/gradients_1/policy/main_graph_0/hidden_1/BiasAdd_grad/BiasAddGrad
�
Eoptimizer/gradients_1/policy/main_graph_0/hidden_1/MatMul_grad/MatMulMatMulXoptimizer/gradients_1/policy/main_graph_0/hidden_1/BiasAdd_grad/tuple/control_dependency(policy/main_graph_0/hidden_1/kernel/read*
T0*
transpose_a( *
transpose_b(
�
Goptimizer/gradients_1/policy/main_graph_0/hidden_1/MatMul_grad/MatMul_1MatMul policy/main_graph_0/hidden_0/MulXoptimizer/gradients_1/policy/main_graph_0/hidden_1/BiasAdd_grad/tuple/control_dependency*
T0*
transpose_a(*
transpose_b( 
�
Ooptimizer/gradients_1/policy/main_graph_0/hidden_1/MatMul_grad/tuple/group_depsNoOpF^optimizer/gradients_1/policy/main_graph_0/hidden_1/MatMul_grad/MatMulH^optimizer/gradients_1/policy/main_graph_0/hidden_1/MatMul_grad/MatMul_1
�
Woptimizer/gradients_1/policy/main_graph_0/hidden_1/MatMul_grad/tuple/control_dependencyIdentityEoptimizer/gradients_1/policy/main_graph_0/hidden_1/MatMul_grad/MatMulP^optimizer/gradients_1/policy/main_graph_0/hidden_1/MatMul_grad/tuple/group_deps*
T0*X
_classN
LJloc:@optimizer/gradients_1/policy/main_graph_0/hidden_1/MatMul_grad/MatMul
�
Yoptimizer/gradients_1/policy/main_graph_0/hidden_1/MatMul_grad/tuple/control_dependency_1IdentityGoptimizer/gradients_1/policy/main_graph_0/hidden_1/MatMul_grad/MatMul_1P^optimizer/gradients_1/policy/main_graph_0/hidden_1/MatMul_grad/tuple/group_deps*
T0*Z
_classP
NLloc:@optimizer/gradients_1/policy/main_graph_0/hidden_1/MatMul_grad/MatMul_1
�
Aoptimizer/gradients_1/policy/main_graph_0/hidden_0/Mul_grad/ShapeShape$policy/main_graph_0/hidden_0/BiasAdd*
T0*
out_type0
�
Coptimizer/gradients_1/policy/main_graph_0/hidden_0/Mul_grad/Shape_1Shape$policy/main_graph_0/hidden_0/Sigmoid*
T0*
out_type0
�
Qoptimizer/gradients_1/policy/main_graph_0/hidden_0/Mul_grad/BroadcastGradientArgsBroadcastGradientArgsAoptimizer/gradients_1/policy/main_graph_0/hidden_0/Mul_grad/ShapeCoptimizer/gradients_1/policy/main_graph_0/hidden_0/Mul_grad/Shape_1*
T0
�
?optimizer/gradients_1/policy/main_graph_0/hidden_0/Mul_grad/MulMulWoptimizer/gradients_1/policy/main_graph_0/hidden_1/MatMul_grad/tuple/control_dependency$policy/main_graph_0/hidden_0/Sigmoid*
T0
�
?optimizer/gradients_1/policy/main_graph_0/hidden_0/Mul_grad/SumSum?optimizer/gradients_1/policy/main_graph_0/hidden_0/Mul_grad/MulQoptimizer/gradients_1/policy/main_graph_0/hidden_0/Mul_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
Coptimizer/gradients_1/policy/main_graph_0/hidden_0/Mul_grad/ReshapeReshape?optimizer/gradients_1/policy/main_graph_0/hidden_0/Mul_grad/SumAoptimizer/gradients_1/policy/main_graph_0/hidden_0/Mul_grad/Shape*
T0*
Tshape0
�
Aoptimizer/gradients_1/policy/main_graph_0/hidden_0/Mul_grad/Mul_1Mul$policy/main_graph_0/hidden_0/BiasAddWoptimizer/gradients_1/policy/main_graph_0/hidden_1/MatMul_grad/tuple/control_dependency*
T0
�
Aoptimizer/gradients_1/policy/main_graph_0/hidden_0/Mul_grad/Sum_1SumAoptimizer/gradients_1/policy/main_graph_0/hidden_0/Mul_grad/Mul_1Soptimizer/gradients_1/policy/main_graph_0/hidden_0/Mul_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
Eoptimizer/gradients_1/policy/main_graph_0/hidden_0/Mul_grad/Reshape_1ReshapeAoptimizer/gradients_1/policy/main_graph_0/hidden_0/Mul_grad/Sum_1Coptimizer/gradients_1/policy/main_graph_0/hidden_0/Mul_grad/Shape_1*
T0*
Tshape0
�
Loptimizer/gradients_1/policy/main_graph_0/hidden_0/Mul_grad/tuple/group_depsNoOpD^optimizer/gradients_1/policy/main_graph_0/hidden_0/Mul_grad/ReshapeF^optimizer/gradients_1/policy/main_graph_0/hidden_0/Mul_grad/Reshape_1
�
Toptimizer/gradients_1/policy/main_graph_0/hidden_0/Mul_grad/tuple/control_dependencyIdentityCoptimizer/gradients_1/policy/main_graph_0/hidden_0/Mul_grad/ReshapeM^optimizer/gradients_1/policy/main_graph_0/hidden_0/Mul_grad/tuple/group_deps*
T0*V
_classL
JHloc:@optimizer/gradients_1/policy/main_graph_0/hidden_0/Mul_grad/Reshape
�
Voptimizer/gradients_1/policy/main_graph_0/hidden_0/Mul_grad/tuple/control_dependency_1IdentityEoptimizer/gradients_1/policy/main_graph_0/hidden_0/Mul_grad/Reshape_1M^optimizer/gradients_1/policy/main_graph_0/hidden_0/Mul_grad/tuple/group_deps*
T0*X
_classN
LJloc:@optimizer/gradients_1/policy/main_graph_0/hidden_0/Mul_grad/Reshape_1
�
Koptimizer/gradients_1/policy/main_graph_0/hidden_0/Sigmoid_grad/SigmoidGradSigmoidGrad$policy/main_graph_0/hidden_0/SigmoidVoptimizer/gradients_1/policy/main_graph_0/hidden_0/Mul_grad/tuple/control_dependency_1*
T0
�
optimizer/gradients_1/AddN_11AddNToptimizer/gradients_1/policy/main_graph_0/hidden_0/Mul_grad/tuple/control_dependencyKoptimizer/gradients_1/policy/main_graph_0/hidden_0/Sigmoid_grad/SigmoidGrad*
N*
T0*V
_classL
JHloc:@optimizer/gradients_1/policy/main_graph_0/hidden_0/Mul_grad/Reshape
�
Koptimizer/gradients_1/policy/main_graph_0/hidden_0/BiasAdd_grad/BiasAddGradBiasAddGradoptimizer/gradients_1/AddN_11*
T0*
data_formatNHWC
�
Poptimizer/gradients_1/policy/main_graph_0/hidden_0/BiasAdd_grad/tuple/group_depsNoOp^optimizer/gradients_1/AddN_11L^optimizer/gradients_1/policy/main_graph_0/hidden_0/BiasAdd_grad/BiasAddGrad
�
Xoptimizer/gradients_1/policy/main_graph_0/hidden_0/BiasAdd_grad/tuple/control_dependencyIdentityoptimizer/gradients_1/AddN_11Q^optimizer/gradients_1/policy/main_graph_0/hidden_0/BiasAdd_grad/tuple/group_deps*
T0*V
_classL
JHloc:@optimizer/gradients_1/policy/main_graph_0/hidden_0/Mul_grad/Reshape
�
Zoptimizer/gradients_1/policy/main_graph_0/hidden_0/BiasAdd_grad/tuple/control_dependency_1IdentityKoptimizer/gradients_1/policy/main_graph_0/hidden_0/BiasAdd_grad/BiasAddGradQ^optimizer/gradients_1/policy/main_graph_0/hidden_0/BiasAdd_grad/tuple/group_deps*
T0*^
_classT
RPloc:@optimizer/gradients_1/policy/main_graph_0/hidden_0/BiasAdd_grad/BiasAddGrad
�
Eoptimizer/gradients_1/policy/main_graph_0/hidden_0/MatMul_grad/MatMulMatMulXoptimizer/gradients_1/policy/main_graph_0/hidden_0/BiasAdd_grad/tuple/control_dependency(policy/main_graph_0/hidden_0/kernel/read*
T0*
transpose_a( *
transpose_b(
�
Goptimizer/gradients_1/policy/main_graph_0/hidden_0/MatMul_grad/MatMul_1MatMulvector_observationXoptimizer/gradients_1/policy/main_graph_0/hidden_0/BiasAdd_grad/tuple/control_dependency*
T0*
transpose_a(*
transpose_b( 
�
Ooptimizer/gradients_1/policy/main_graph_0/hidden_0/MatMul_grad/tuple/group_depsNoOpF^optimizer/gradients_1/policy/main_graph_0/hidden_0/MatMul_grad/MatMulH^optimizer/gradients_1/policy/main_graph_0/hidden_0/MatMul_grad/MatMul_1
�
Woptimizer/gradients_1/policy/main_graph_0/hidden_0/MatMul_grad/tuple/control_dependencyIdentityEoptimizer/gradients_1/policy/main_graph_0/hidden_0/MatMul_grad/MatMulP^optimizer/gradients_1/policy/main_graph_0/hidden_0/MatMul_grad/tuple/group_deps*
T0*X
_classN
LJloc:@optimizer/gradients_1/policy/main_graph_0/hidden_0/MatMul_grad/MatMul
�
Yoptimizer/gradients_1/policy/main_graph_0/hidden_0/MatMul_grad/tuple/control_dependency_1IdentityGoptimizer/gradients_1/policy/main_graph_0/hidden_0/MatMul_grad/MatMul_1P^optimizer/gradients_1/policy/main_graph_0/hidden_0/MatMul_grad/tuple/group_deps*
T0*Z
_classP
NLloc:@optimizer/gradients_1/policy/main_graph_0/hidden_0/MatMul_grad/MatMul_1
D
optimizer/gradients_2/ShapeConst*
dtype0*
valueB 
L
optimizer/gradients_2/grad_ys_0Const*
dtype0*
valueB
 *  �?
{
optimizer/gradients_2/FillFilloptimizer/gradients_2/Shapeoptimizer/gradients_2/grad_ys_0*
T0*

index_type0
Z
.optimizer/gradients_2/optimizer/sub_4_grad/NegNegoptimizer/gradients_2/Fill*
T0
�
;optimizer/gradients_2/optimizer/sub_4_grad/tuple/group_depsNoOp^optimizer/gradients_2/Fill/^optimizer/gradients_2/optimizer/sub_4_grad/Neg
�
Coptimizer/gradients_2/optimizer/sub_4_grad/tuple/control_dependencyIdentityoptimizer/gradients_2/Fill<^optimizer/gradients_2/optimizer/sub_4_grad/tuple/group_deps*
T0*-
_class#
!loc:@optimizer/gradients_2/Fill
�
Eoptimizer/gradients_2/optimizer/sub_4_grad/tuple/control_dependency_1Identity.optimizer/gradients_2/optimizer/sub_4_grad/Neg<^optimizer/gradients_2/optimizer/sub_4_grad/tuple/group_deps*
T0*A
_class7
53loc:@optimizer/gradients_2/optimizer/sub_4_grad/Neg
�
<optimizer/gradients_2/optimizer/add_11_grad/tuple/group_depsNoOpD^optimizer/gradients_2/optimizer/sub_4_grad/tuple/control_dependency
�
Doptimizer/gradients_2/optimizer/add_11_grad/tuple/control_dependencyIdentityCoptimizer/gradients_2/optimizer/sub_4_grad/tuple/control_dependency=^optimizer/gradients_2/optimizer/add_11_grad/tuple/group_deps*
T0*-
_class#
!loc:@optimizer/gradients_2/Fill
�
Foptimizer/gradients_2/optimizer/add_11_grad/tuple/control_dependency_1IdentityCoptimizer/gradients_2/optimizer/sub_4_grad/tuple/control_dependency=^optimizer/gradients_2/optimizer/add_11_grad/tuple/group_deps*
T0*-
_class#
!loc:@optimizer/gradients_2/Fill
�
/optimizer/gradients_2/optimizer/mul_10_grad/MulMulEoptimizer/gradients_2/optimizer/sub_4_grad/tuple/control_dependency_1optimizer/Mean_7*
T0
�
1optimizer/gradients_2/optimizer/mul_10_grad/Mul_1MulEoptimizer/gradients_2/optimizer/sub_4_grad/tuple/control_dependency_1optimizer/Variable_2/read*
T0
�
<optimizer/gradients_2/optimizer/mul_10_grad/tuple/group_depsNoOp0^optimizer/gradients_2/optimizer/mul_10_grad/Mul2^optimizer/gradients_2/optimizer/mul_10_grad/Mul_1
�
Doptimizer/gradients_2/optimizer/mul_10_grad/tuple/control_dependencyIdentity/optimizer/gradients_2/optimizer/mul_10_grad/Mul=^optimizer/gradients_2/optimizer/mul_10_grad/tuple/group_deps*
T0*B
_class8
64loc:@optimizer/gradients_2/optimizer/mul_10_grad/Mul
�
Foptimizer/gradients_2/optimizer/mul_10_grad/tuple/control_dependency_1Identity1optimizer/gradients_2/optimizer/mul_10_grad/Mul_1=^optimizer/gradients_2/optimizer/mul_10_grad/tuple/group_deps*
T0*D
_class:
86loc:@optimizer/gradients_2/optimizer/mul_10_grad/Mul_1
�
.optimizer/gradients_2/optimizer/Neg_5_grad/NegNegDoptimizer/gradients_2/optimizer/add_11_grad/tuple/control_dependency*
T0
�
.optimizer/gradients_2/optimizer/mul_9_grad/MulMulFoptimizer/gradients_2/optimizer/add_11_grad/tuple/control_dependency_1optimizer/Mean_5*
T0
�
0optimizer/gradients_2/optimizer/mul_9_grad/Mul_1MulFoptimizer/gradients_2/optimizer/add_11_grad/tuple/control_dependency_1optimizer/mul_9/x*
T0
�
;optimizer/gradients_2/optimizer/mul_9_grad/tuple/group_depsNoOp/^optimizer/gradients_2/optimizer/mul_9_grad/Mul1^optimizer/gradients_2/optimizer/mul_9_grad/Mul_1
�
Coptimizer/gradients_2/optimizer/mul_9_grad/tuple/control_dependencyIdentity.optimizer/gradients_2/optimizer/mul_9_grad/Mul<^optimizer/gradients_2/optimizer/mul_9_grad/tuple/group_deps*
T0*A
_class7
53loc:@optimizer/gradients_2/optimizer/mul_9_grad/Mul
�
Eoptimizer/gradients_2/optimizer/mul_9_grad/tuple/control_dependency_1Identity0optimizer/gradients_2/optimizer/mul_9_grad/Mul_1<^optimizer/gradients_2/optimizer/mul_9_grad/tuple/group_deps*
T0*C
_class9
75loc:@optimizer/gradients_2/optimizer/mul_9_grad/Mul_1
g
9optimizer/gradients_2/optimizer/Mean_7_grad/Reshape/shapeConst*
dtype0*
valueB:
�
3optimizer/gradients_2/optimizer/Mean_7_grad/ReshapeReshapeFoptimizer/gradients_2/optimizer/mul_10_grad/tuple/control_dependency_19optimizer/gradients_2/optimizer/Mean_7_grad/Reshape/shape*
T0*
Tshape0
s
1optimizer/gradients_2/optimizer/Mean_7_grad/ShapeShapeoptimizer/DynamicPartition_5:1*
T0*
out_type0
�
0optimizer/gradients_2/optimizer/Mean_7_grad/TileTile3optimizer/gradients_2/optimizer/Mean_7_grad/Reshape1optimizer/gradients_2/optimizer/Mean_7_grad/Shape*
T0*

Tmultiples0
u
3optimizer/gradients_2/optimizer/Mean_7_grad/Shape_1Shapeoptimizer/DynamicPartition_5:1*
T0*
out_type0
\
3optimizer/gradients_2/optimizer/Mean_7_grad/Shape_2Const*
dtype0*
valueB 
_
1optimizer/gradients_2/optimizer/Mean_7_grad/ConstConst*
dtype0*
valueB: 
�
0optimizer/gradients_2/optimizer/Mean_7_grad/ProdProd3optimizer/gradients_2/optimizer/Mean_7_grad/Shape_11optimizer/gradients_2/optimizer/Mean_7_grad/Const*
T0*

Tidx0*
	keep_dims( 
a
3optimizer/gradients_2/optimizer/Mean_7_grad/Const_1Const*
dtype0*
valueB: 
�
2optimizer/gradients_2/optimizer/Mean_7_grad/Prod_1Prod3optimizer/gradients_2/optimizer/Mean_7_grad/Shape_23optimizer/gradients_2/optimizer/Mean_7_grad/Const_1*
T0*

Tidx0*
	keep_dims( 
_
5optimizer/gradients_2/optimizer/Mean_7_grad/Maximum/yConst*
dtype0*
value	B :
�
3optimizer/gradients_2/optimizer/Mean_7_grad/MaximumMaximum2optimizer/gradients_2/optimizer/Mean_7_grad/Prod_15optimizer/gradients_2/optimizer/Mean_7_grad/Maximum/y*
T0
�
4optimizer/gradients_2/optimizer/Mean_7_grad/floordivFloorDiv0optimizer/gradients_2/optimizer/Mean_7_grad/Prod3optimizer/gradients_2/optimizer/Mean_7_grad/Maximum*
T0
�
0optimizer/gradients_2/optimizer/Mean_7_grad/CastCast4optimizer/gradients_2/optimizer/Mean_7_grad/floordiv*

DstT0*

SrcT0*
Truncate( 
�
3optimizer/gradients_2/optimizer/Mean_7_grad/truedivRealDiv0optimizer/gradients_2/optimizer/Mean_7_grad/Tile0optimizer/gradients_2/optimizer/Mean_7_grad/Cast*
T0
n
9optimizer/gradients_2/optimizer/Mean_6_grad/Reshape/shapeConst*
dtype0*
valueB"      
�
3optimizer/gradients_2/optimizer/Mean_6_grad/ReshapeReshape.optimizer/gradients_2/optimizer/Neg_5_grad/Neg9optimizer/gradients_2/optimizer/Mean_6_grad/Reshape/shape*
T0*
Tshape0
s
1optimizer/gradients_2/optimizer/Mean_6_grad/ShapeShapeoptimizer/DynamicPartition_4:1*
T0*
out_type0
�
0optimizer/gradients_2/optimizer/Mean_6_grad/TileTile3optimizer/gradients_2/optimizer/Mean_6_grad/Reshape1optimizer/gradients_2/optimizer/Mean_6_grad/Shape*
T0*

Tmultiples0
u
3optimizer/gradients_2/optimizer/Mean_6_grad/Shape_1Shapeoptimizer/DynamicPartition_4:1*
T0*
out_type0
\
3optimizer/gradients_2/optimizer/Mean_6_grad/Shape_2Const*
dtype0*
valueB 
_
1optimizer/gradients_2/optimizer/Mean_6_grad/ConstConst*
dtype0*
valueB: 
�
0optimizer/gradients_2/optimizer/Mean_6_grad/ProdProd3optimizer/gradients_2/optimizer/Mean_6_grad/Shape_11optimizer/gradients_2/optimizer/Mean_6_grad/Const*
T0*

Tidx0*
	keep_dims( 
a
3optimizer/gradients_2/optimizer/Mean_6_grad/Const_1Const*
dtype0*
valueB: 
�
2optimizer/gradients_2/optimizer/Mean_6_grad/Prod_1Prod3optimizer/gradients_2/optimizer/Mean_6_grad/Shape_23optimizer/gradients_2/optimizer/Mean_6_grad/Const_1*
T0*

Tidx0*
	keep_dims( 
_
5optimizer/gradients_2/optimizer/Mean_6_grad/Maximum/yConst*
dtype0*
value	B :
�
3optimizer/gradients_2/optimizer/Mean_6_grad/MaximumMaximum2optimizer/gradients_2/optimizer/Mean_6_grad/Prod_15optimizer/gradients_2/optimizer/Mean_6_grad/Maximum/y*
T0
�
4optimizer/gradients_2/optimizer/Mean_6_grad/floordivFloorDiv0optimizer/gradients_2/optimizer/Mean_6_grad/Prod3optimizer/gradients_2/optimizer/Mean_6_grad/Maximum*
T0
�
0optimizer/gradients_2/optimizer/Mean_6_grad/CastCast4optimizer/gradients_2/optimizer/Mean_6_grad/floordiv*

DstT0*

SrcT0*
Truncate( 
�
3optimizer/gradients_2/optimizer/Mean_6_grad/truedivRealDiv0optimizer/gradients_2/optimizer/Mean_6_grad/Tile0optimizer/gradients_2/optimizer/Mean_6_grad/Cast*
T0
g
9optimizer/gradients_2/optimizer/Mean_5_grad/Reshape/shapeConst*
dtype0*
valueB:
�
3optimizer/gradients_2/optimizer/Mean_5_grad/ReshapeReshapeEoptimizer/gradients_2/optimizer/mul_9_grad/tuple/control_dependency_19optimizer/gradients_2/optimizer/Mean_5_grad/Reshape/shape*
T0*
Tshape0
_
1optimizer/gradients_2/optimizer/Mean_5_grad/ConstConst*
dtype0*
valueB:
�
0optimizer/gradients_2/optimizer/Mean_5_grad/TileTile3optimizer/gradients_2/optimizer/Mean_5_grad/Reshape1optimizer/gradients_2/optimizer/Mean_5_grad/Const*
T0*

Tmultiples0
`
3optimizer/gradients_2/optimizer/Mean_5_grad/Const_1Const*
dtype0*
valueB
 *   @
�
3optimizer/gradients_2/optimizer/Mean_5_grad/truedivRealDiv0optimizer/gradients_2/optimizer/Mean_5_grad/Tile3optimizer/gradients_2/optimizer/Mean_5_grad/Const_1*
T0
T
 optimizer/gradients_2/zeros_like	ZerosLikeoptimizer/DynamicPartition_5*
T0
e
=optimizer/gradients_2/optimizer/DynamicPartition_5_grad/ShapeShapeCast*
T0*
out_type0
k
=optimizer/gradients_2/optimizer/DynamicPartition_5_grad/ConstConst*
dtype0*
valueB: 
�
<optimizer/gradients_2/optimizer/DynamicPartition_5_grad/ProdProd=optimizer/gradients_2/optimizer/DynamicPartition_5_grad/Shape=optimizer/gradients_2/optimizer/DynamicPartition_5_grad/Const*
T0*

Tidx0*
	keep_dims( 
m
Coptimizer/gradients_2/optimizer/DynamicPartition_5_grad/range/startConst*
dtype0*
value	B : 
m
Coptimizer/gradients_2/optimizer/DynamicPartition_5_grad/range/deltaConst*
dtype0*
value	B :
�
=optimizer/gradients_2/optimizer/DynamicPartition_5_grad/rangeRangeCoptimizer/gradients_2/optimizer/DynamicPartition_5_grad/range/start<optimizer/gradients_2/optimizer/DynamicPartition_5_grad/ProdCoptimizer/gradients_2/optimizer/DynamicPartition_5_grad/range/delta*

Tidx0
�
?optimizer/gradients_2/optimizer/DynamicPartition_5_grad/ReshapeReshape=optimizer/gradients_2/optimizer/DynamicPartition_5_grad/range=optimizer/gradients_2/optimizer/DynamicPartition_5_grad/Shape*
T0*
Tshape0
�
Hoptimizer/gradients_2/optimizer/DynamicPartition_5_grad/DynamicPartitionDynamicPartition?optimizer/gradients_2/optimizer/DynamicPartition_5_grad/ReshapeCast*
T0*
num_partitions
�
Moptimizer/gradients_2/optimizer/DynamicPartition_5_grad/ParallelDynamicStitchParallelDynamicStitchHoptimizer/gradients_2/optimizer/DynamicPartition_5_grad/DynamicPartitionJoptimizer/gradients_2/optimizer/DynamicPartition_5_grad/DynamicPartition:1 optimizer/gradients_2/zeros_like3optimizer/gradients_2/optimizer/Mean_7_grad/truediv*
N*
T0
q
?optimizer/gradients_2/optimizer/DynamicPartition_5_grad/Shape_1Shapepolicy_1/Sum_2*
T0*
out_type0
�
Aoptimizer/gradients_2/optimizer/DynamicPartition_5_grad/Reshape_1ReshapeMoptimizer/gradients_2/optimizer/DynamicPartition_5_grad/ParallelDynamicStitch?optimizer/gradients_2/optimizer/DynamicPartition_5_grad/Shape_1*
T0*
Tshape0
V
"optimizer/gradients_2/zeros_like_1	ZerosLikeoptimizer/DynamicPartition_4*
T0
e
=optimizer/gradients_2/optimizer/DynamicPartition_4_grad/ShapeShapeCast*
T0*
out_type0
k
=optimizer/gradients_2/optimizer/DynamicPartition_4_grad/ConstConst*
dtype0*
valueB: 
�
<optimizer/gradients_2/optimizer/DynamicPartition_4_grad/ProdProd=optimizer/gradients_2/optimizer/DynamicPartition_4_grad/Shape=optimizer/gradients_2/optimizer/DynamicPartition_4_grad/Const*
T0*

Tidx0*
	keep_dims( 
m
Coptimizer/gradients_2/optimizer/DynamicPartition_4_grad/range/startConst*
dtype0*
value	B : 
m
Coptimizer/gradients_2/optimizer/DynamicPartition_4_grad/range/deltaConst*
dtype0*
value	B :
�
=optimizer/gradients_2/optimizer/DynamicPartition_4_grad/rangeRangeCoptimizer/gradients_2/optimizer/DynamicPartition_4_grad/range/start<optimizer/gradients_2/optimizer/DynamicPartition_4_grad/ProdCoptimizer/gradients_2/optimizer/DynamicPartition_4_grad/range/delta*

Tidx0
�
?optimizer/gradients_2/optimizer/DynamicPartition_4_grad/ReshapeReshape=optimizer/gradients_2/optimizer/DynamicPartition_4_grad/range=optimizer/gradients_2/optimizer/DynamicPartition_4_grad/Shape*
T0*
Tshape0
�
Hoptimizer/gradients_2/optimizer/DynamicPartition_4_grad/DynamicPartitionDynamicPartition?optimizer/gradients_2/optimizer/DynamicPartition_4_grad/ReshapeCast*
T0*
num_partitions
�
Moptimizer/gradients_2/optimizer/DynamicPartition_4_grad/ParallelDynamicStitchParallelDynamicStitchHoptimizer/gradients_2/optimizer/DynamicPartition_4_grad/DynamicPartitionJoptimizer/gradients_2/optimizer/DynamicPartition_4_grad/DynamicPartition:1"optimizer/gradients_2/zeros_like_13optimizer/gradients_2/optimizer/Mean_6_grad/truediv*
N*
T0
t
?optimizer/gradients_2/optimizer/DynamicPartition_4_grad/Shape_1Shapeoptimizer/Minimum*
T0*
out_type0
�
Aoptimizer/gradients_2/optimizer/DynamicPartition_4_grad/Reshape_1ReshapeMoptimizer/gradients_2/optimizer/DynamicPartition_4_grad/ParallelDynamicStitch?optimizer/gradients_2/optimizer/DynamicPartition_4_grad/Shape_1*
T0*
Tshape0
�
9optimizer/gradients_2/optimizer/Mean_5/input_grad/unstackUnpack3optimizer/gradients_2/optimizer/Mean_5_grad/truediv*
T0*

axis *	
num
�
Boptimizer/gradients_2/optimizer/Mean_5/input_grad/tuple/group_depsNoOp:^optimizer/gradients_2/optimizer/Mean_5/input_grad/unstack
�
Joptimizer/gradients_2/optimizer/Mean_5/input_grad/tuple/control_dependencyIdentity9optimizer/gradients_2/optimizer/Mean_5/input_grad/unstackC^optimizer/gradients_2/optimizer/Mean_5/input_grad/tuple/group_deps*
T0*L
_classB
@>loc:@optimizer/gradients_2/optimizer/Mean_5/input_grad/unstack
�
Loptimizer/gradients_2/optimizer/Mean_5/input_grad/tuple/control_dependency_1Identity;optimizer/gradients_2/optimizer/Mean_5/input_grad/unstack:1C^optimizer/gradients_2/optimizer/Mean_5/input_grad/tuple/group_deps*
T0*L
_classB
@>loc:@optimizer/gradients_2/optimizer/Mean_5/input_grad/unstack
a
/optimizer/gradients_2/policy_1/Sum_2_grad/ShapeShapepolicy_1/stack*
T0*
out_type0
�
.optimizer/gradients_2/policy_1/Sum_2_grad/SizeConst*B
_class8
64loc:@optimizer/gradients_2/policy_1/Sum_2_grad/Shape*
dtype0*
value	B :
�
-optimizer/gradients_2/policy_1/Sum_2_grad/addAddV2 policy_1/Sum_2/reduction_indices.optimizer/gradients_2/policy_1/Sum_2_grad/Size*
T0*B
_class8
64loc:@optimizer/gradients_2/policy_1/Sum_2_grad/Shape
�
-optimizer/gradients_2/policy_1/Sum_2_grad/modFloorMod-optimizer/gradients_2/policy_1/Sum_2_grad/add.optimizer/gradients_2/policy_1/Sum_2_grad/Size*
T0*B
_class8
64loc:@optimizer/gradients_2/policy_1/Sum_2_grad/Shape
�
1optimizer/gradients_2/policy_1/Sum_2_grad/Shape_1Const*B
_class8
64loc:@optimizer/gradients_2/policy_1/Sum_2_grad/Shape*
dtype0*
valueB 
�
5optimizer/gradients_2/policy_1/Sum_2_grad/range/startConst*B
_class8
64loc:@optimizer/gradients_2/policy_1/Sum_2_grad/Shape*
dtype0*
value	B : 
�
5optimizer/gradients_2/policy_1/Sum_2_grad/range/deltaConst*B
_class8
64loc:@optimizer/gradients_2/policy_1/Sum_2_grad/Shape*
dtype0*
value	B :
�
/optimizer/gradients_2/policy_1/Sum_2_grad/rangeRange5optimizer/gradients_2/policy_1/Sum_2_grad/range/start.optimizer/gradients_2/policy_1/Sum_2_grad/Size5optimizer/gradients_2/policy_1/Sum_2_grad/range/delta*

Tidx0*B
_class8
64loc:@optimizer/gradients_2/policy_1/Sum_2_grad/Shape
�
4optimizer/gradients_2/policy_1/Sum_2_grad/Fill/valueConst*B
_class8
64loc:@optimizer/gradients_2/policy_1/Sum_2_grad/Shape*
dtype0*
value	B :
�
.optimizer/gradients_2/policy_1/Sum_2_grad/FillFill1optimizer/gradients_2/policy_1/Sum_2_grad/Shape_14optimizer/gradients_2/policy_1/Sum_2_grad/Fill/value*
T0*B
_class8
64loc:@optimizer/gradients_2/policy_1/Sum_2_grad/Shape*

index_type0
�
7optimizer/gradients_2/policy_1/Sum_2_grad/DynamicStitchDynamicStitch/optimizer/gradients_2/policy_1/Sum_2_grad/range-optimizer/gradients_2/policy_1/Sum_2_grad/mod/optimizer/gradients_2/policy_1/Sum_2_grad/Shape.optimizer/gradients_2/policy_1/Sum_2_grad/Fill*
N*
T0*B
_class8
64loc:@optimizer/gradients_2/policy_1/Sum_2_grad/Shape
�
1optimizer/gradients_2/policy_1/Sum_2_grad/ReshapeReshapeAoptimizer/gradients_2/optimizer/DynamicPartition_5_grad/Reshape_17optimizer/gradients_2/policy_1/Sum_2_grad/DynamicStitch*
T0*
Tshape0
�
5optimizer/gradients_2/policy_1/Sum_2_grad/BroadcastToBroadcastTo1optimizer/gradients_2/policy_1/Sum_2_grad/Reshape/optimizer/gradients_2/policy_1/Sum_2_grad/Shape*
T0*

Tidx0
e
2optimizer/gradients_2/optimizer/Minimum_grad/ShapeShapeoptimizer/mul_7*
T0*
out_type0
g
4optimizer/gradients_2/optimizer/Minimum_grad/Shape_1Shapeoptimizer/mul_8*
T0*
out_type0
�
4optimizer/gradients_2/optimizer/Minimum_grad/Shape_2ShapeAoptimizer/gradients_2/optimizer/DynamicPartition_4_grad/Reshape_1*
T0*
out_type0
e
8optimizer/gradients_2/optimizer/Minimum_grad/zeros/ConstConst*
dtype0*
valueB
 *    
�
2optimizer/gradients_2/optimizer/Minimum_grad/zerosFill4optimizer/gradients_2/optimizer/Minimum_grad/Shape_28optimizer/gradients_2/optimizer/Minimum_grad/zeros/Const*
T0*

index_type0
n
6optimizer/gradients_2/optimizer/Minimum_grad/LessEqual	LessEqualoptimizer/mul_7optimizer/mul_8*
T0
�
Boptimizer/gradients_2/optimizer/Minimum_grad/BroadcastGradientArgsBroadcastGradientArgs2optimizer/gradients_2/optimizer/Minimum_grad/Shape4optimizer/gradients_2/optimizer/Minimum_grad/Shape_1*
T0
�
5optimizer/gradients_2/optimizer/Minimum_grad/SelectV2SelectV26optimizer/gradients_2/optimizer/Minimum_grad/LessEqualAoptimizer/gradients_2/optimizer/DynamicPartition_4_grad/Reshape_12optimizer/gradients_2/optimizer/Minimum_grad/zeros*
T0
�
0optimizer/gradients_2/optimizer/Minimum_grad/SumSum5optimizer/gradients_2/optimizer/Minimum_grad/SelectV2Boptimizer/gradients_2/optimizer/Minimum_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
4optimizer/gradients_2/optimizer/Minimum_grad/ReshapeReshape0optimizer/gradients_2/optimizer/Minimum_grad/Sum2optimizer/gradients_2/optimizer/Minimum_grad/Shape*
T0*
Tshape0
�
7optimizer/gradients_2/optimizer/Minimum_grad/SelectV2_1SelectV26optimizer/gradients_2/optimizer/Minimum_grad/LessEqual2optimizer/gradients_2/optimizer/Minimum_grad/zerosAoptimizer/gradients_2/optimizer/DynamicPartition_4_grad/Reshape_1*
T0
�
2optimizer/gradients_2/optimizer/Minimum_grad/Sum_1Sum7optimizer/gradients_2/optimizer/Minimum_grad/SelectV2_1Doptimizer/gradients_2/optimizer/Minimum_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
6optimizer/gradients_2/optimizer/Minimum_grad/Reshape_1Reshape2optimizer/gradients_2/optimizer/Minimum_grad/Sum_14optimizer/gradients_2/optimizer/Minimum_grad/Shape_1*
T0*
Tshape0
�
=optimizer/gradients_2/optimizer/Minimum_grad/tuple/group_depsNoOp5^optimizer/gradients_2/optimizer/Minimum_grad/Reshape7^optimizer/gradients_2/optimizer/Minimum_grad/Reshape_1
�
Eoptimizer/gradients_2/optimizer/Minimum_grad/tuple/control_dependencyIdentity4optimizer/gradients_2/optimizer/Minimum_grad/Reshape>^optimizer/gradients_2/optimizer/Minimum_grad/tuple/group_deps*
T0*G
_class=
;9loc:@optimizer/gradients_2/optimizer/Minimum_grad/Reshape
�
Goptimizer/gradients_2/optimizer/Minimum_grad/tuple/control_dependency_1Identity6optimizer/gradients_2/optimizer/Minimum_grad/Reshape_1>^optimizer/gradients_2/optimizer/Minimum_grad/tuple/group_deps*
T0*I
_class?
=;loc:@optimizer/gradients_2/optimizer/Minimum_grad/Reshape_1
g
9optimizer/gradients_2/optimizer/Mean_3_grad/Reshape/shapeConst*
dtype0*
valueB:
�
3optimizer/gradients_2/optimizer/Mean_3_grad/ReshapeReshapeJoptimizer/gradients_2/optimizer/Mean_5/input_grad/tuple/control_dependency9optimizer/gradients_2/optimizer/Mean_3_grad/Reshape/shape*
T0*
Tshape0
s
1optimizer/gradients_2/optimizer/Mean_3_grad/ShapeShapeoptimizer/DynamicPartition_2:1*
T0*
out_type0
�
0optimizer/gradients_2/optimizer/Mean_3_grad/TileTile3optimizer/gradients_2/optimizer/Mean_3_grad/Reshape1optimizer/gradients_2/optimizer/Mean_3_grad/Shape*
T0*

Tmultiples0
u
3optimizer/gradients_2/optimizer/Mean_3_grad/Shape_1Shapeoptimizer/DynamicPartition_2:1*
T0*
out_type0
\
3optimizer/gradients_2/optimizer/Mean_3_grad/Shape_2Const*
dtype0*
valueB 
_
1optimizer/gradients_2/optimizer/Mean_3_grad/ConstConst*
dtype0*
valueB: 
�
0optimizer/gradients_2/optimizer/Mean_3_grad/ProdProd3optimizer/gradients_2/optimizer/Mean_3_grad/Shape_11optimizer/gradients_2/optimizer/Mean_3_grad/Const*
T0*

Tidx0*
	keep_dims( 
a
3optimizer/gradients_2/optimizer/Mean_3_grad/Const_1Const*
dtype0*
valueB: 
�
2optimizer/gradients_2/optimizer/Mean_3_grad/Prod_1Prod3optimizer/gradients_2/optimizer/Mean_3_grad/Shape_23optimizer/gradients_2/optimizer/Mean_3_grad/Const_1*
T0*

Tidx0*
	keep_dims( 
_
5optimizer/gradients_2/optimizer/Mean_3_grad/Maximum/yConst*
dtype0*
value	B :
�
3optimizer/gradients_2/optimizer/Mean_3_grad/MaximumMaximum2optimizer/gradients_2/optimizer/Mean_3_grad/Prod_15optimizer/gradients_2/optimizer/Mean_3_grad/Maximum/y*
T0
�
4optimizer/gradients_2/optimizer/Mean_3_grad/floordivFloorDiv0optimizer/gradients_2/optimizer/Mean_3_grad/Prod3optimizer/gradients_2/optimizer/Mean_3_grad/Maximum*
T0
�
0optimizer/gradients_2/optimizer/Mean_3_grad/CastCast4optimizer/gradients_2/optimizer/Mean_3_grad/floordiv*

DstT0*

SrcT0*
Truncate( 
�
3optimizer/gradients_2/optimizer/Mean_3_grad/truedivRealDiv0optimizer/gradients_2/optimizer/Mean_3_grad/Tile0optimizer/gradients_2/optimizer/Mean_3_grad/Cast*
T0
g
9optimizer/gradients_2/optimizer/Mean_4_grad/Reshape/shapeConst*
dtype0*
valueB:
�
3optimizer/gradients_2/optimizer/Mean_4_grad/ReshapeReshapeLoptimizer/gradients_2/optimizer/Mean_5/input_grad/tuple/control_dependency_19optimizer/gradients_2/optimizer/Mean_4_grad/Reshape/shape*
T0*
Tshape0
s
1optimizer/gradients_2/optimizer/Mean_4_grad/ShapeShapeoptimizer/DynamicPartition_3:1*
T0*
out_type0
�
0optimizer/gradients_2/optimizer/Mean_4_grad/TileTile3optimizer/gradients_2/optimizer/Mean_4_grad/Reshape1optimizer/gradients_2/optimizer/Mean_4_grad/Shape*
T0*

Tmultiples0
u
3optimizer/gradients_2/optimizer/Mean_4_grad/Shape_1Shapeoptimizer/DynamicPartition_3:1*
T0*
out_type0
\
3optimizer/gradients_2/optimizer/Mean_4_grad/Shape_2Const*
dtype0*
valueB 
_
1optimizer/gradients_2/optimizer/Mean_4_grad/ConstConst*
dtype0*
valueB: 
�
0optimizer/gradients_2/optimizer/Mean_4_grad/ProdProd3optimizer/gradients_2/optimizer/Mean_4_grad/Shape_11optimizer/gradients_2/optimizer/Mean_4_grad/Const*
T0*

Tidx0*
	keep_dims( 
a
3optimizer/gradients_2/optimizer/Mean_4_grad/Const_1Const*
dtype0*
valueB: 
�
2optimizer/gradients_2/optimizer/Mean_4_grad/Prod_1Prod3optimizer/gradients_2/optimizer/Mean_4_grad/Shape_23optimizer/gradients_2/optimizer/Mean_4_grad/Const_1*
T0*

Tidx0*
	keep_dims( 
_
5optimizer/gradients_2/optimizer/Mean_4_grad/Maximum/yConst*
dtype0*
value	B :
�
3optimizer/gradients_2/optimizer/Mean_4_grad/MaximumMaximum2optimizer/gradients_2/optimizer/Mean_4_grad/Prod_15optimizer/gradients_2/optimizer/Mean_4_grad/Maximum/y*
T0
�
4optimizer/gradients_2/optimizer/Mean_4_grad/floordivFloorDiv0optimizer/gradients_2/optimizer/Mean_4_grad/Prod3optimizer/gradients_2/optimizer/Mean_4_grad/Maximum*
T0
�
0optimizer/gradients_2/optimizer/Mean_4_grad/CastCast4optimizer/gradients_2/optimizer/Mean_4_grad/floordiv*

DstT0*

SrcT0*
Truncate( 
�
3optimizer/gradients_2/optimizer/Mean_4_grad/truedivRealDiv0optimizer/gradients_2/optimizer/Mean_4_grad/Tile0optimizer/gradients_2/optimizer/Mean_4_grad/Cast*
T0
�
1optimizer/gradients_2/policy_1/stack_grad/unstackUnpack5optimizer/gradients_2/policy_1/Sum_2_grad/BroadcastTo*
T0*

axis*	
num
v
:optimizer/gradients_2/policy_1/stack_grad/tuple/group_depsNoOp2^optimizer/gradients_2/policy_1/stack_grad/unstack
�
Boptimizer/gradients_2/policy_1/stack_grad/tuple/control_dependencyIdentity1optimizer/gradients_2/policy_1/stack_grad/unstack;^optimizer/gradients_2/policy_1/stack_grad/tuple/group_deps*
T0*D
_class:
86loc:@optimizer/gradients_2/policy_1/stack_grad/unstack
�
Doptimizer/gradients_2/policy_1/stack_grad/tuple/control_dependency_1Identity3optimizer/gradients_2/policy_1/stack_grad/unstack:1;^optimizer/gradients_2/policy_1/stack_grad/tuple/group_deps*
T0*D
_class:
86loc:@optimizer/gradients_2/policy_1/stack_grad/unstack
a
0optimizer/gradients_2/optimizer/mul_7_grad/ShapeShapeoptimizer/Exp*
T0*
out_type0
j
2optimizer/gradients_2/optimizer/mul_7_grad/Shape_1Shapeoptimizer/ExpandDims*
T0*
out_type0
�
@optimizer/gradients_2/optimizer/mul_7_grad/BroadcastGradientArgsBroadcastGradientArgs0optimizer/gradients_2/optimizer/mul_7_grad/Shape2optimizer/gradients_2/optimizer/mul_7_grad/Shape_1*
T0
�
.optimizer/gradients_2/optimizer/mul_7_grad/MulMulEoptimizer/gradients_2/optimizer/Minimum_grad/tuple/control_dependencyoptimizer/ExpandDims*
T0
�
.optimizer/gradients_2/optimizer/mul_7_grad/SumSum.optimizer/gradients_2/optimizer/mul_7_grad/Mul@optimizer/gradients_2/optimizer/mul_7_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
2optimizer/gradients_2/optimizer/mul_7_grad/ReshapeReshape.optimizer/gradients_2/optimizer/mul_7_grad/Sum0optimizer/gradients_2/optimizer/mul_7_grad/Shape*
T0*
Tshape0
�
0optimizer/gradients_2/optimizer/mul_7_grad/Mul_1Muloptimizer/ExpEoptimizer/gradients_2/optimizer/Minimum_grad/tuple/control_dependency*
T0
�
0optimizer/gradients_2/optimizer/mul_7_grad/Sum_1Sum0optimizer/gradients_2/optimizer/mul_7_grad/Mul_1Boptimizer/gradients_2/optimizer/mul_7_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
4optimizer/gradients_2/optimizer/mul_7_grad/Reshape_1Reshape0optimizer/gradients_2/optimizer/mul_7_grad/Sum_12optimizer/gradients_2/optimizer/mul_7_grad/Shape_1*
T0*
Tshape0
�
;optimizer/gradients_2/optimizer/mul_7_grad/tuple/group_depsNoOp3^optimizer/gradients_2/optimizer/mul_7_grad/Reshape5^optimizer/gradients_2/optimizer/mul_7_grad/Reshape_1
�
Coptimizer/gradients_2/optimizer/mul_7_grad/tuple/control_dependencyIdentity2optimizer/gradients_2/optimizer/mul_7_grad/Reshape<^optimizer/gradients_2/optimizer/mul_7_grad/tuple/group_deps*
T0*E
_class;
97loc:@optimizer/gradients_2/optimizer/mul_7_grad/Reshape
�
Eoptimizer/gradients_2/optimizer/mul_7_grad/tuple/control_dependency_1Identity4optimizer/gradients_2/optimizer/mul_7_grad/Reshape_1<^optimizer/gradients_2/optimizer/mul_7_grad/tuple/group_deps*
T0*G
_class=
;9loc:@optimizer/gradients_2/optimizer/mul_7_grad/Reshape_1
m
0optimizer/gradients_2/optimizer/mul_8_grad/ShapeShapeoptimizer/clip_by_value_2*
T0*
out_type0
j
2optimizer/gradients_2/optimizer/mul_8_grad/Shape_1Shapeoptimizer/ExpandDims*
T0*
out_type0
�
@optimizer/gradients_2/optimizer/mul_8_grad/BroadcastGradientArgsBroadcastGradientArgs0optimizer/gradients_2/optimizer/mul_8_grad/Shape2optimizer/gradients_2/optimizer/mul_8_grad/Shape_1*
T0
�
.optimizer/gradients_2/optimizer/mul_8_grad/MulMulGoptimizer/gradients_2/optimizer/Minimum_grad/tuple/control_dependency_1optimizer/ExpandDims*
T0
�
.optimizer/gradients_2/optimizer/mul_8_grad/SumSum.optimizer/gradients_2/optimizer/mul_8_grad/Mul@optimizer/gradients_2/optimizer/mul_8_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
2optimizer/gradients_2/optimizer/mul_8_grad/ReshapeReshape.optimizer/gradients_2/optimizer/mul_8_grad/Sum0optimizer/gradients_2/optimizer/mul_8_grad/Shape*
T0*
Tshape0
�
0optimizer/gradients_2/optimizer/mul_8_grad/Mul_1Muloptimizer/clip_by_value_2Goptimizer/gradients_2/optimizer/Minimum_grad/tuple/control_dependency_1*
T0
�
0optimizer/gradients_2/optimizer/mul_8_grad/Sum_1Sum0optimizer/gradients_2/optimizer/mul_8_grad/Mul_1Boptimizer/gradients_2/optimizer/mul_8_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
4optimizer/gradients_2/optimizer/mul_8_grad/Reshape_1Reshape0optimizer/gradients_2/optimizer/mul_8_grad/Sum_12optimizer/gradients_2/optimizer/mul_8_grad/Shape_1*
T0*
Tshape0
�
;optimizer/gradients_2/optimizer/mul_8_grad/tuple/group_depsNoOp3^optimizer/gradients_2/optimizer/mul_8_grad/Reshape5^optimizer/gradients_2/optimizer/mul_8_grad/Reshape_1
�
Coptimizer/gradients_2/optimizer/mul_8_grad/tuple/control_dependencyIdentity2optimizer/gradients_2/optimizer/mul_8_grad/Reshape<^optimizer/gradients_2/optimizer/mul_8_grad/tuple/group_deps*
T0*E
_class;
97loc:@optimizer/gradients_2/optimizer/mul_8_grad/Reshape
�
Eoptimizer/gradients_2/optimizer/mul_8_grad/tuple/control_dependency_1Identity4optimizer/gradients_2/optimizer/mul_8_grad/Reshape_1<^optimizer/gradients_2/optimizer/mul_8_grad/tuple/group_deps*
T0*G
_class=
;9loc:@optimizer/gradients_2/optimizer/mul_8_grad/Reshape_1
V
"optimizer/gradients_2/zeros_like_2	ZerosLikeoptimizer/DynamicPartition_2*
T0
e
=optimizer/gradients_2/optimizer/DynamicPartition_2_grad/ShapeShapeCast*
T0*
out_type0
k
=optimizer/gradients_2/optimizer/DynamicPartition_2_grad/ConstConst*
dtype0*
valueB: 
�
<optimizer/gradients_2/optimizer/DynamicPartition_2_grad/ProdProd=optimizer/gradients_2/optimizer/DynamicPartition_2_grad/Shape=optimizer/gradients_2/optimizer/DynamicPartition_2_grad/Const*
T0*

Tidx0*
	keep_dims( 
m
Coptimizer/gradients_2/optimizer/DynamicPartition_2_grad/range/startConst*
dtype0*
value	B : 
m
Coptimizer/gradients_2/optimizer/DynamicPartition_2_grad/range/deltaConst*
dtype0*
value	B :
�
=optimizer/gradients_2/optimizer/DynamicPartition_2_grad/rangeRangeCoptimizer/gradients_2/optimizer/DynamicPartition_2_grad/range/start<optimizer/gradients_2/optimizer/DynamicPartition_2_grad/ProdCoptimizer/gradients_2/optimizer/DynamicPartition_2_grad/range/delta*

Tidx0
�
?optimizer/gradients_2/optimizer/DynamicPartition_2_grad/ReshapeReshape=optimizer/gradients_2/optimizer/DynamicPartition_2_grad/range=optimizer/gradients_2/optimizer/DynamicPartition_2_grad/Shape*
T0*
Tshape0
�
Hoptimizer/gradients_2/optimizer/DynamicPartition_2_grad/DynamicPartitionDynamicPartition?optimizer/gradients_2/optimizer/DynamicPartition_2_grad/ReshapeCast*
T0*
num_partitions
�
Moptimizer/gradients_2/optimizer/DynamicPartition_2_grad/ParallelDynamicStitchParallelDynamicStitchHoptimizer/gradients_2/optimizer/DynamicPartition_2_grad/DynamicPartitionJoptimizer/gradients_2/optimizer/DynamicPartition_2_grad/DynamicPartition:1"optimizer/gradients_2/zeros_like_23optimizer/gradients_2/optimizer/Mean_3_grad/truediv*
N*
T0
t
?optimizer/gradients_2/optimizer/DynamicPartition_2_grad/Shape_1Shapeoptimizer/Maximum*
T0*
out_type0
�
Aoptimizer/gradients_2/optimizer/DynamicPartition_2_grad/Reshape_1ReshapeMoptimizer/gradients_2/optimizer/DynamicPartition_2_grad/ParallelDynamicStitch?optimizer/gradients_2/optimizer/DynamicPartition_2_grad/Shape_1*
T0*
Tshape0
V
"optimizer/gradients_2/zeros_like_3	ZerosLikeoptimizer/DynamicPartition_3*
T0
e
=optimizer/gradients_2/optimizer/DynamicPartition_3_grad/ShapeShapeCast*
T0*
out_type0
k
=optimizer/gradients_2/optimizer/DynamicPartition_3_grad/ConstConst*
dtype0*
valueB: 
�
<optimizer/gradients_2/optimizer/DynamicPartition_3_grad/ProdProd=optimizer/gradients_2/optimizer/DynamicPartition_3_grad/Shape=optimizer/gradients_2/optimizer/DynamicPartition_3_grad/Const*
T0*

Tidx0*
	keep_dims( 
m
Coptimizer/gradients_2/optimizer/DynamicPartition_3_grad/range/startConst*
dtype0*
value	B : 
m
Coptimizer/gradients_2/optimizer/DynamicPartition_3_grad/range/deltaConst*
dtype0*
value	B :
�
=optimizer/gradients_2/optimizer/DynamicPartition_3_grad/rangeRangeCoptimizer/gradients_2/optimizer/DynamicPartition_3_grad/range/start<optimizer/gradients_2/optimizer/DynamicPartition_3_grad/ProdCoptimizer/gradients_2/optimizer/DynamicPartition_3_grad/range/delta*

Tidx0
�
?optimizer/gradients_2/optimizer/DynamicPartition_3_grad/ReshapeReshape=optimizer/gradients_2/optimizer/DynamicPartition_3_grad/range=optimizer/gradients_2/optimizer/DynamicPartition_3_grad/Shape*
T0*
Tshape0
�
Hoptimizer/gradients_2/optimizer/DynamicPartition_3_grad/DynamicPartitionDynamicPartition?optimizer/gradients_2/optimizer/DynamicPartition_3_grad/ReshapeCast*
T0*
num_partitions
�
Moptimizer/gradients_2/optimizer/DynamicPartition_3_grad/ParallelDynamicStitchParallelDynamicStitchHoptimizer/gradients_2/optimizer/DynamicPartition_3_grad/DynamicPartitionJoptimizer/gradients_2/optimizer/DynamicPartition_3_grad/DynamicPartition:1"optimizer/gradients_2/zeros_like_33optimizer/gradients_2/optimizer/Mean_4_grad/truediv*
N*
T0
v
?optimizer/gradients_2/optimizer/DynamicPartition_3_grad/Shape_1Shapeoptimizer/Maximum_1*
T0*
out_type0
�
Aoptimizer/gradients_2/optimizer/DynamicPartition_3_grad/Reshape_1ReshapeMoptimizer/gradients_2/optimizer/DynamicPartition_3_grad/ParallelDynamicStitch?optimizer/gradients_2/optimizer/DynamicPartition_3_grad/Shape_1*
T0*
Tshape0
�
Uoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits/Reshape_2_grad/ShapeShape*policy_1/softmax_cross_entropy_with_logits*
T0*
out_type0
�
Woptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits/Reshape_2_grad/ReshapeReshapeBoptimizer/gradients_2/policy_1/stack_grad/tuple/control_dependencyUoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits/Reshape_2_grad/Shape*
T0*
Tshape0
�
Woptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_1/Reshape_2_grad/ShapeShape,policy_1/softmax_cross_entropy_with_logits_1*
T0*
out_type0
�
Yoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_1/Reshape_2_grad/ReshapeReshapeDoptimizer/gradients_2/policy_1/stack_grad/tuple/control_dependency_1Woptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_1/Reshape_2_grad/Shape*
T0*
Tshape0

:optimizer/gradients_2/optimizer/clip_by_value_2_grad/ShapeShape!optimizer/clip_by_value_2/Minimum*
T0*
out_type0
e
<optimizer/gradients_2/optimizer/clip_by_value_2_grad/Shape_1Const*
dtype0*
valueB 
�
<optimizer/gradients_2/optimizer/clip_by_value_2_grad/Shape_2ShapeCoptimizer/gradients_2/optimizer/mul_8_grad/tuple/control_dependency*
T0*
out_type0
m
@optimizer/gradients_2/optimizer/clip_by_value_2_grad/zeros/ConstConst*
dtype0*
valueB
 *    
�
:optimizer/gradients_2/optimizer/clip_by_value_2_grad/zerosFill<optimizer/gradients_2/optimizer/clip_by_value_2_grad/Shape_2@optimizer/gradients_2/optimizer/clip_by_value_2_grad/zeros/Const*
T0*

index_type0
�
Aoptimizer/gradients_2/optimizer/clip_by_value_2_grad/GreaterEqualGreaterEqual!optimizer/clip_by_value_2/Minimumoptimizer/sub_3*
T0
�
Joptimizer/gradients_2/optimizer/clip_by_value_2_grad/BroadcastGradientArgsBroadcastGradientArgs:optimizer/gradients_2/optimizer/clip_by_value_2_grad/Shape<optimizer/gradients_2/optimizer/clip_by_value_2_grad/Shape_1*
T0
�
=optimizer/gradients_2/optimizer/clip_by_value_2_grad/SelectV2SelectV2Aoptimizer/gradients_2/optimizer/clip_by_value_2_grad/GreaterEqualCoptimizer/gradients_2/optimizer/mul_8_grad/tuple/control_dependency:optimizer/gradients_2/optimizer/clip_by_value_2_grad/zeros*
T0
�
8optimizer/gradients_2/optimizer/clip_by_value_2_grad/SumSum=optimizer/gradients_2/optimizer/clip_by_value_2_grad/SelectV2Joptimizer/gradients_2/optimizer/clip_by_value_2_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
<optimizer/gradients_2/optimizer/clip_by_value_2_grad/ReshapeReshape8optimizer/gradients_2/optimizer/clip_by_value_2_grad/Sum:optimizer/gradients_2/optimizer/clip_by_value_2_grad/Shape*
T0*
Tshape0
�
?optimizer/gradients_2/optimizer/clip_by_value_2_grad/SelectV2_1SelectV2Aoptimizer/gradients_2/optimizer/clip_by_value_2_grad/GreaterEqual:optimizer/gradients_2/optimizer/clip_by_value_2_grad/zerosCoptimizer/gradients_2/optimizer/mul_8_grad/tuple/control_dependency*
T0
�
:optimizer/gradients_2/optimizer/clip_by_value_2_grad/Sum_1Sum?optimizer/gradients_2/optimizer/clip_by_value_2_grad/SelectV2_1Loptimizer/gradients_2/optimizer/clip_by_value_2_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
>optimizer/gradients_2/optimizer/clip_by_value_2_grad/Reshape_1Reshape:optimizer/gradients_2/optimizer/clip_by_value_2_grad/Sum_1<optimizer/gradients_2/optimizer/clip_by_value_2_grad/Shape_1*
T0*
Tshape0
�
Eoptimizer/gradients_2/optimizer/clip_by_value_2_grad/tuple/group_depsNoOp=^optimizer/gradients_2/optimizer/clip_by_value_2_grad/Reshape?^optimizer/gradients_2/optimizer/clip_by_value_2_grad/Reshape_1
�
Moptimizer/gradients_2/optimizer/clip_by_value_2_grad/tuple/control_dependencyIdentity<optimizer/gradients_2/optimizer/clip_by_value_2_grad/ReshapeF^optimizer/gradients_2/optimizer/clip_by_value_2_grad/tuple/group_deps*
T0*O
_classE
CAloc:@optimizer/gradients_2/optimizer/clip_by_value_2_grad/Reshape
�
Ooptimizer/gradients_2/optimizer/clip_by_value_2_grad/tuple/control_dependency_1Identity>optimizer/gradients_2/optimizer/clip_by_value_2_grad/Reshape_1F^optimizer/gradients_2/optimizer/clip_by_value_2_grad/tuple/group_deps*
T0*Q
_classG
ECloc:@optimizer/gradients_2/optimizer/clip_by_value_2_grad/Reshape_1
s
2optimizer/gradients_2/optimizer/Maximum_grad/ShapeShapeoptimizer/SquaredDifference_1*
T0*
out_type0
u
4optimizer/gradients_2/optimizer/Maximum_grad/Shape_1Shapeoptimizer/SquaredDifference_2*
T0*
out_type0
�
4optimizer/gradients_2/optimizer/Maximum_grad/Shape_2ShapeAoptimizer/gradients_2/optimizer/DynamicPartition_2_grad/Reshape_1*
T0*
out_type0
e
8optimizer/gradients_2/optimizer/Maximum_grad/zeros/ConstConst*
dtype0*
valueB
 *    
�
2optimizer/gradients_2/optimizer/Maximum_grad/zerosFill4optimizer/gradients_2/optimizer/Maximum_grad/Shape_28optimizer/gradients_2/optimizer/Maximum_grad/zeros/Const*
T0*

index_type0
�
9optimizer/gradients_2/optimizer/Maximum_grad/GreaterEqualGreaterEqualoptimizer/SquaredDifference_1optimizer/SquaredDifference_2*
T0
�
Boptimizer/gradients_2/optimizer/Maximum_grad/BroadcastGradientArgsBroadcastGradientArgs2optimizer/gradients_2/optimizer/Maximum_grad/Shape4optimizer/gradients_2/optimizer/Maximum_grad/Shape_1*
T0
�
5optimizer/gradients_2/optimizer/Maximum_grad/SelectV2SelectV29optimizer/gradients_2/optimizer/Maximum_grad/GreaterEqualAoptimizer/gradients_2/optimizer/DynamicPartition_2_grad/Reshape_12optimizer/gradients_2/optimizer/Maximum_grad/zeros*
T0
�
0optimizer/gradients_2/optimizer/Maximum_grad/SumSum5optimizer/gradients_2/optimizer/Maximum_grad/SelectV2Boptimizer/gradients_2/optimizer/Maximum_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
4optimizer/gradients_2/optimizer/Maximum_grad/ReshapeReshape0optimizer/gradients_2/optimizer/Maximum_grad/Sum2optimizer/gradients_2/optimizer/Maximum_grad/Shape*
T0*
Tshape0
�
7optimizer/gradients_2/optimizer/Maximum_grad/SelectV2_1SelectV29optimizer/gradients_2/optimizer/Maximum_grad/GreaterEqual2optimizer/gradients_2/optimizer/Maximum_grad/zerosAoptimizer/gradients_2/optimizer/DynamicPartition_2_grad/Reshape_1*
T0
�
2optimizer/gradients_2/optimizer/Maximum_grad/Sum_1Sum7optimizer/gradients_2/optimizer/Maximum_grad/SelectV2_1Doptimizer/gradients_2/optimizer/Maximum_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
6optimizer/gradients_2/optimizer/Maximum_grad/Reshape_1Reshape2optimizer/gradients_2/optimizer/Maximum_grad/Sum_14optimizer/gradients_2/optimizer/Maximum_grad/Shape_1*
T0*
Tshape0
�
=optimizer/gradients_2/optimizer/Maximum_grad/tuple/group_depsNoOp5^optimizer/gradients_2/optimizer/Maximum_grad/Reshape7^optimizer/gradients_2/optimizer/Maximum_grad/Reshape_1
�
Eoptimizer/gradients_2/optimizer/Maximum_grad/tuple/control_dependencyIdentity4optimizer/gradients_2/optimizer/Maximum_grad/Reshape>^optimizer/gradients_2/optimizer/Maximum_grad/tuple/group_deps*
T0*G
_class=
;9loc:@optimizer/gradients_2/optimizer/Maximum_grad/Reshape
�
Goptimizer/gradients_2/optimizer/Maximum_grad/tuple/control_dependency_1Identity6optimizer/gradients_2/optimizer/Maximum_grad/Reshape_1>^optimizer/gradients_2/optimizer/Maximum_grad/tuple/group_deps*
T0*I
_class?
=;loc:@optimizer/gradients_2/optimizer/Maximum_grad/Reshape_1
u
4optimizer/gradients_2/optimizer/Maximum_1_grad/ShapeShapeoptimizer/SquaredDifference_3*
T0*
out_type0
w
6optimizer/gradients_2/optimizer/Maximum_1_grad/Shape_1Shapeoptimizer/SquaredDifference_4*
T0*
out_type0
�
6optimizer/gradients_2/optimizer/Maximum_1_grad/Shape_2ShapeAoptimizer/gradients_2/optimizer/DynamicPartition_3_grad/Reshape_1*
T0*
out_type0
g
:optimizer/gradients_2/optimizer/Maximum_1_grad/zeros/ConstConst*
dtype0*
valueB
 *    
�
4optimizer/gradients_2/optimizer/Maximum_1_grad/zerosFill6optimizer/gradients_2/optimizer/Maximum_1_grad/Shape_2:optimizer/gradients_2/optimizer/Maximum_1_grad/zeros/Const*
T0*

index_type0
�
;optimizer/gradients_2/optimizer/Maximum_1_grad/GreaterEqualGreaterEqualoptimizer/SquaredDifference_3optimizer/SquaredDifference_4*
T0
�
Doptimizer/gradients_2/optimizer/Maximum_1_grad/BroadcastGradientArgsBroadcastGradientArgs4optimizer/gradients_2/optimizer/Maximum_1_grad/Shape6optimizer/gradients_2/optimizer/Maximum_1_grad/Shape_1*
T0
�
7optimizer/gradients_2/optimizer/Maximum_1_grad/SelectV2SelectV2;optimizer/gradients_2/optimizer/Maximum_1_grad/GreaterEqualAoptimizer/gradients_2/optimizer/DynamicPartition_3_grad/Reshape_14optimizer/gradients_2/optimizer/Maximum_1_grad/zeros*
T0
�
2optimizer/gradients_2/optimizer/Maximum_1_grad/SumSum7optimizer/gradients_2/optimizer/Maximum_1_grad/SelectV2Doptimizer/gradients_2/optimizer/Maximum_1_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
6optimizer/gradients_2/optimizer/Maximum_1_grad/ReshapeReshape2optimizer/gradients_2/optimizer/Maximum_1_grad/Sum4optimizer/gradients_2/optimizer/Maximum_1_grad/Shape*
T0*
Tshape0
�
9optimizer/gradients_2/optimizer/Maximum_1_grad/SelectV2_1SelectV2;optimizer/gradients_2/optimizer/Maximum_1_grad/GreaterEqual4optimizer/gradients_2/optimizer/Maximum_1_grad/zerosAoptimizer/gradients_2/optimizer/DynamicPartition_3_grad/Reshape_1*
T0
�
4optimizer/gradients_2/optimizer/Maximum_1_grad/Sum_1Sum9optimizer/gradients_2/optimizer/Maximum_1_grad/SelectV2_1Foptimizer/gradients_2/optimizer/Maximum_1_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
8optimizer/gradients_2/optimizer/Maximum_1_grad/Reshape_1Reshape4optimizer/gradients_2/optimizer/Maximum_1_grad/Sum_16optimizer/gradients_2/optimizer/Maximum_1_grad/Shape_1*
T0*
Tshape0
�
?optimizer/gradients_2/optimizer/Maximum_1_grad/tuple/group_depsNoOp7^optimizer/gradients_2/optimizer/Maximum_1_grad/Reshape9^optimizer/gradients_2/optimizer/Maximum_1_grad/Reshape_1
�
Goptimizer/gradients_2/optimizer/Maximum_1_grad/tuple/control_dependencyIdentity6optimizer/gradients_2/optimizer/Maximum_1_grad/Reshape@^optimizer/gradients_2/optimizer/Maximum_1_grad/tuple/group_deps*
T0*I
_class?
=;loc:@optimizer/gradients_2/optimizer/Maximum_1_grad/Reshape
�
Ioptimizer/gradients_2/optimizer/Maximum_1_grad/tuple/control_dependency_1Identity8optimizer/gradients_2/optimizer/Maximum_1_grad/Reshape_1@^optimizer/gradients_2/optimizer/Maximum_1_grad/tuple/group_deps*
T0*K
_classA
?=loc:@optimizer/gradients_2/optimizer/Maximum_1_grad/Reshape_1
f
"optimizer/gradients_2/zeros_like_4	ZerosLike,policy_1/softmax_cross_entropy_with_logits:1*
T0
�
Toptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_grad/ExpandDims/dimConst*
dtype0*
valueB :
���������
�
Poptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_grad/ExpandDims
ExpandDimsWoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits/Reshape_2_grad/ReshapeToptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_grad/ExpandDims/dim*
T0*

Tdim0
�
Ioptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_grad/mulMulPoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_grad/ExpandDims,policy_1/softmax_cross_entropy_with_logits:1*
T0
�
Poptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_grad/LogSoftmax
LogSoftmax2policy_1/softmax_cross_entropy_with_logits/Reshape*
T0
�
Ioptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_grad/NegNegPoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_grad/LogSoftmax*
T0
�
Voptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_grad/ExpandDims_1/dimConst*
dtype0*
valueB :
���������
�
Roptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_grad/ExpandDims_1
ExpandDimsWoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits/Reshape_2_grad/ReshapeVoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_grad/ExpandDims_1/dim*
T0*

Tdim0
�
Koptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_grad/mul_1MulRoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_grad/ExpandDims_1Ioptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_grad/Neg*
T0
�
Voptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_grad/tuple/group_depsNoOpJ^optimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_grad/mulL^optimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_grad/mul_1
�
^optimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_grad/tuple/control_dependencyIdentityIoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_grad/mulW^optimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_grad/tuple/group_deps*
T0*\
_classR
PNloc:@optimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_grad/mul
�
`optimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_grad/tuple/control_dependency_1IdentityKoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_grad/mul_1W^optimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_grad/tuple/group_deps*
T0*^
_classT
RPloc:@optimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_grad/mul_1
h
"optimizer/gradients_2/zeros_like_5	ZerosLike.policy_1/softmax_cross_entropy_with_logits_1:1*
T0
�
Voptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_1_grad/ExpandDims/dimConst*
dtype0*
valueB :
���������
�
Roptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_1_grad/ExpandDims
ExpandDimsYoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_1/Reshape_2_grad/ReshapeVoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_1_grad/ExpandDims/dim*
T0*

Tdim0
�
Koptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_1_grad/mulMulRoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_1_grad/ExpandDims.policy_1/softmax_cross_entropy_with_logits_1:1*
T0
�
Roptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_1_grad/LogSoftmax
LogSoftmax4policy_1/softmax_cross_entropy_with_logits_1/Reshape*
T0
�
Koptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_1_grad/NegNegRoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_1_grad/LogSoftmax*
T0
�
Xoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_1_grad/ExpandDims_1/dimConst*
dtype0*
valueB :
���������
�
Toptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_1_grad/ExpandDims_1
ExpandDimsYoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_1/Reshape_2_grad/ReshapeXoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_1_grad/ExpandDims_1/dim*
T0*

Tdim0
�
Moptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_1_grad/mul_1MulToptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_1_grad/ExpandDims_1Koptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_1_grad/Neg*
T0
�
Xoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_1_grad/tuple/group_depsNoOpL^optimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_1_grad/mulN^optimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_1_grad/mul_1
�
`optimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_1_grad/tuple/control_dependencyIdentityKoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_1_grad/mulY^optimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_1_grad/tuple/group_deps*
T0*^
_classT
RPloc:@optimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_1_grad/mul
�
boptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_1_grad/tuple/control_dependency_1IdentityMoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_1_grad/mul_1Y^optimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_1_grad/tuple/group_deps*
T0*`
_classV
TRloc:@optimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_1_grad/mul_1
s
Boptimizer/gradients_2/optimizer/clip_by_value_2/Minimum_grad/ShapeShapeoptimizer/Exp*
T0*
out_type0
m
Doptimizer/gradients_2/optimizer/clip_by_value_2/Minimum_grad/Shape_1Const*
dtype0*
valueB 
�
Doptimizer/gradients_2/optimizer/clip_by_value_2/Minimum_grad/Shape_2ShapeMoptimizer/gradients_2/optimizer/clip_by_value_2_grad/tuple/control_dependency*
T0*
out_type0
u
Hoptimizer/gradients_2/optimizer/clip_by_value_2/Minimum_grad/zeros/ConstConst*
dtype0*
valueB
 *    
�
Boptimizer/gradients_2/optimizer/clip_by_value_2/Minimum_grad/zerosFillDoptimizer/gradients_2/optimizer/clip_by_value_2/Minimum_grad/Shape_2Hoptimizer/gradients_2/optimizer/clip_by_value_2/Minimum_grad/zeros/Const*
T0*

index_type0
}
Foptimizer/gradients_2/optimizer/clip_by_value_2/Minimum_grad/LessEqual	LessEqualoptimizer/Expoptimizer/add_10*
T0
�
Roptimizer/gradients_2/optimizer/clip_by_value_2/Minimum_grad/BroadcastGradientArgsBroadcastGradientArgsBoptimizer/gradients_2/optimizer/clip_by_value_2/Minimum_grad/ShapeDoptimizer/gradients_2/optimizer/clip_by_value_2/Minimum_grad/Shape_1*
T0
�
Eoptimizer/gradients_2/optimizer/clip_by_value_2/Minimum_grad/SelectV2SelectV2Foptimizer/gradients_2/optimizer/clip_by_value_2/Minimum_grad/LessEqualMoptimizer/gradients_2/optimizer/clip_by_value_2_grad/tuple/control_dependencyBoptimizer/gradients_2/optimizer/clip_by_value_2/Minimum_grad/zeros*
T0
�
@optimizer/gradients_2/optimizer/clip_by_value_2/Minimum_grad/SumSumEoptimizer/gradients_2/optimizer/clip_by_value_2/Minimum_grad/SelectV2Roptimizer/gradients_2/optimizer/clip_by_value_2/Minimum_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
Doptimizer/gradients_2/optimizer/clip_by_value_2/Minimum_grad/ReshapeReshape@optimizer/gradients_2/optimizer/clip_by_value_2/Minimum_grad/SumBoptimizer/gradients_2/optimizer/clip_by_value_2/Minimum_grad/Shape*
T0*
Tshape0
�
Goptimizer/gradients_2/optimizer/clip_by_value_2/Minimum_grad/SelectV2_1SelectV2Foptimizer/gradients_2/optimizer/clip_by_value_2/Minimum_grad/LessEqualBoptimizer/gradients_2/optimizer/clip_by_value_2/Minimum_grad/zerosMoptimizer/gradients_2/optimizer/clip_by_value_2_grad/tuple/control_dependency*
T0
�
Boptimizer/gradients_2/optimizer/clip_by_value_2/Minimum_grad/Sum_1SumGoptimizer/gradients_2/optimizer/clip_by_value_2/Minimum_grad/SelectV2_1Toptimizer/gradients_2/optimizer/clip_by_value_2/Minimum_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
Foptimizer/gradients_2/optimizer/clip_by_value_2/Minimum_grad/Reshape_1ReshapeBoptimizer/gradients_2/optimizer/clip_by_value_2/Minimum_grad/Sum_1Doptimizer/gradients_2/optimizer/clip_by_value_2/Minimum_grad/Shape_1*
T0*
Tshape0
�
Moptimizer/gradients_2/optimizer/clip_by_value_2/Minimum_grad/tuple/group_depsNoOpE^optimizer/gradients_2/optimizer/clip_by_value_2/Minimum_grad/ReshapeG^optimizer/gradients_2/optimizer/clip_by_value_2/Minimum_grad/Reshape_1
�
Uoptimizer/gradients_2/optimizer/clip_by_value_2/Minimum_grad/tuple/control_dependencyIdentityDoptimizer/gradients_2/optimizer/clip_by_value_2/Minimum_grad/ReshapeN^optimizer/gradients_2/optimizer/clip_by_value_2/Minimum_grad/tuple/group_deps*
T0*W
_classM
KIloc:@optimizer/gradients_2/optimizer/clip_by_value_2/Minimum_grad/Reshape
�
Woptimizer/gradients_2/optimizer/clip_by_value_2/Minimum_grad/tuple/control_dependency_1IdentityFoptimizer/gradients_2/optimizer/clip_by_value_2/Minimum_grad/Reshape_1N^optimizer/gradients_2/optimizer/clip_by_value_2/Minimum_grad/tuple/group_deps*
T0*Y
_classO
MKloc:@optimizer/gradients_2/optimizer/clip_by_value_2/Minimum_grad/Reshape_1
�
?optimizer/gradients_2/optimizer/SquaredDifference_1_grad/scalarConstF^optimizer/gradients_2/optimizer/Maximum_grad/tuple/control_dependency*
dtype0*
valueB
 *   @
�
<optimizer/gradients_2/optimizer/SquaredDifference_1_grad/MulMul?optimizer/gradients_2/optimizer/SquaredDifference_1_grad/scalarEoptimizer/gradients_2/optimizer/Maximum_grad/tuple/control_dependency*
T0
�
<optimizer/gradients_2/optimizer/SquaredDifference_1_grad/subSuboptimizer/extrinsic_returnsoptimizer/Sum_6F^optimizer/gradients_2/optimizer/Maximum_grad/tuple/control_dependency*
T0
�
>optimizer/gradients_2/optimizer/SquaredDifference_1_grad/mul_1Mul<optimizer/gradients_2/optimizer/SquaredDifference_1_grad/Mul<optimizer/gradients_2/optimizer/SquaredDifference_1_grad/sub*
T0
}
>optimizer/gradients_2/optimizer/SquaredDifference_1_grad/ShapeShapeoptimizer/extrinsic_returns*
T0*
out_type0
s
@optimizer/gradients_2/optimizer/SquaredDifference_1_grad/Shape_1Shapeoptimizer/Sum_6*
T0*
out_type0
�
Noptimizer/gradients_2/optimizer/SquaredDifference_1_grad/BroadcastGradientArgsBroadcastGradientArgs>optimizer/gradients_2/optimizer/SquaredDifference_1_grad/Shape@optimizer/gradients_2/optimizer/SquaredDifference_1_grad/Shape_1*
T0
�
<optimizer/gradients_2/optimizer/SquaredDifference_1_grad/SumSum>optimizer/gradients_2/optimizer/SquaredDifference_1_grad/mul_1Noptimizer/gradients_2/optimizer/SquaredDifference_1_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
@optimizer/gradients_2/optimizer/SquaredDifference_1_grad/ReshapeReshape<optimizer/gradients_2/optimizer/SquaredDifference_1_grad/Sum>optimizer/gradients_2/optimizer/SquaredDifference_1_grad/Shape*
T0*
Tshape0
�
>optimizer/gradients_2/optimizer/SquaredDifference_1_grad/Sum_1Sum>optimizer/gradients_2/optimizer/SquaredDifference_1_grad/mul_1Poptimizer/gradients_2/optimizer/SquaredDifference_1_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
Boptimizer/gradients_2/optimizer/SquaredDifference_1_grad/Reshape_1Reshape>optimizer/gradients_2/optimizer/SquaredDifference_1_grad/Sum_1@optimizer/gradients_2/optimizer/SquaredDifference_1_grad/Shape_1*
T0*
Tshape0
�
<optimizer/gradients_2/optimizer/SquaredDifference_1_grad/NegNegBoptimizer/gradients_2/optimizer/SquaredDifference_1_grad/Reshape_1*
T0
�
Ioptimizer/gradients_2/optimizer/SquaredDifference_1_grad/tuple/group_depsNoOp=^optimizer/gradients_2/optimizer/SquaredDifference_1_grad/NegA^optimizer/gradients_2/optimizer/SquaredDifference_1_grad/Reshape
�
Qoptimizer/gradients_2/optimizer/SquaredDifference_1_grad/tuple/control_dependencyIdentity@optimizer/gradients_2/optimizer/SquaredDifference_1_grad/ReshapeJ^optimizer/gradients_2/optimizer/SquaredDifference_1_grad/tuple/group_deps*
T0*S
_classI
GEloc:@optimizer/gradients_2/optimizer/SquaredDifference_1_grad/Reshape
�
Soptimizer/gradients_2/optimizer/SquaredDifference_1_grad/tuple/control_dependency_1Identity<optimizer/gradients_2/optimizer/SquaredDifference_1_grad/NegJ^optimizer/gradients_2/optimizer/SquaredDifference_1_grad/tuple/group_deps*
T0*O
_classE
CAloc:@optimizer/gradients_2/optimizer/SquaredDifference_1_grad/Neg
�
?optimizer/gradients_2/optimizer/SquaredDifference_2_grad/scalarConstH^optimizer/gradients_2/optimizer/Maximum_grad/tuple/control_dependency_1*
dtype0*
valueB
 *   @
�
<optimizer/gradients_2/optimizer/SquaredDifference_2_grad/MulMul?optimizer/gradients_2/optimizer/SquaredDifference_2_grad/scalarGoptimizer/gradients_2/optimizer/Maximum_grad/tuple/control_dependency_1*
T0
�
<optimizer/gradients_2/optimizer/SquaredDifference_2_grad/subSuboptimizer/extrinsic_returnsoptimizer/add_8H^optimizer/gradients_2/optimizer/Maximum_grad/tuple/control_dependency_1*
T0
�
>optimizer/gradients_2/optimizer/SquaredDifference_2_grad/mul_1Mul<optimizer/gradients_2/optimizer/SquaredDifference_2_grad/Mul<optimizer/gradients_2/optimizer/SquaredDifference_2_grad/sub*
T0
}
>optimizer/gradients_2/optimizer/SquaredDifference_2_grad/ShapeShapeoptimizer/extrinsic_returns*
T0*
out_type0
s
@optimizer/gradients_2/optimizer/SquaredDifference_2_grad/Shape_1Shapeoptimizer/add_8*
T0*
out_type0
�
Noptimizer/gradients_2/optimizer/SquaredDifference_2_grad/BroadcastGradientArgsBroadcastGradientArgs>optimizer/gradients_2/optimizer/SquaredDifference_2_grad/Shape@optimizer/gradients_2/optimizer/SquaredDifference_2_grad/Shape_1*
T0
�
<optimizer/gradients_2/optimizer/SquaredDifference_2_grad/SumSum>optimizer/gradients_2/optimizer/SquaredDifference_2_grad/mul_1Noptimizer/gradients_2/optimizer/SquaredDifference_2_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
@optimizer/gradients_2/optimizer/SquaredDifference_2_grad/ReshapeReshape<optimizer/gradients_2/optimizer/SquaredDifference_2_grad/Sum>optimizer/gradients_2/optimizer/SquaredDifference_2_grad/Shape*
T0*
Tshape0
�
>optimizer/gradients_2/optimizer/SquaredDifference_2_grad/Sum_1Sum>optimizer/gradients_2/optimizer/SquaredDifference_2_grad/mul_1Poptimizer/gradients_2/optimizer/SquaredDifference_2_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
Boptimizer/gradients_2/optimizer/SquaredDifference_2_grad/Reshape_1Reshape>optimizer/gradients_2/optimizer/SquaredDifference_2_grad/Sum_1@optimizer/gradients_2/optimizer/SquaredDifference_2_grad/Shape_1*
T0*
Tshape0
�
<optimizer/gradients_2/optimizer/SquaredDifference_2_grad/NegNegBoptimizer/gradients_2/optimizer/SquaredDifference_2_grad/Reshape_1*
T0
�
Ioptimizer/gradients_2/optimizer/SquaredDifference_2_grad/tuple/group_depsNoOp=^optimizer/gradients_2/optimizer/SquaredDifference_2_grad/NegA^optimizer/gradients_2/optimizer/SquaredDifference_2_grad/Reshape
�
Qoptimizer/gradients_2/optimizer/SquaredDifference_2_grad/tuple/control_dependencyIdentity@optimizer/gradients_2/optimizer/SquaredDifference_2_grad/ReshapeJ^optimizer/gradients_2/optimizer/SquaredDifference_2_grad/tuple/group_deps*
T0*S
_classI
GEloc:@optimizer/gradients_2/optimizer/SquaredDifference_2_grad/Reshape
�
Soptimizer/gradients_2/optimizer/SquaredDifference_2_grad/tuple/control_dependency_1Identity<optimizer/gradients_2/optimizer/SquaredDifference_2_grad/NegJ^optimizer/gradients_2/optimizer/SquaredDifference_2_grad/tuple/group_deps*
T0*O
_classE
CAloc:@optimizer/gradients_2/optimizer/SquaredDifference_2_grad/Neg
�
?optimizer/gradients_2/optimizer/SquaredDifference_3_grad/scalarConstH^optimizer/gradients_2/optimizer/Maximum_1_grad/tuple/control_dependency*
dtype0*
valueB
 *   @
�
<optimizer/gradients_2/optimizer/SquaredDifference_3_grad/MulMul?optimizer/gradients_2/optimizer/SquaredDifference_3_grad/scalarGoptimizer/gradients_2/optimizer/Maximum_1_grad/tuple/control_dependency*
T0
�
<optimizer/gradients_2/optimizer/SquaredDifference_3_grad/subSuboptimizer/curiosity_returnsoptimizer/Sum_8H^optimizer/gradients_2/optimizer/Maximum_1_grad/tuple/control_dependency*
T0
�
>optimizer/gradients_2/optimizer/SquaredDifference_3_grad/mul_1Mul<optimizer/gradients_2/optimizer/SquaredDifference_3_grad/Mul<optimizer/gradients_2/optimizer/SquaredDifference_3_grad/sub*
T0
}
>optimizer/gradients_2/optimizer/SquaredDifference_3_grad/ShapeShapeoptimizer/curiosity_returns*
T0*
out_type0
s
@optimizer/gradients_2/optimizer/SquaredDifference_3_grad/Shape_1Shapeoptimizer/Sum_8*
T0*
out_type0
�
Noptimizer/gradients_2/optimizer/SquaredDifference_3_grad/BroadcastGradientArgsBroadcastGradientArgs>optimizer/gradients_2/optimizer/SquaredDifference_3_grad/Shape@optimizer/gradients_2/optimizer/SquaredDifference_3_grad/Shape_1*
T0
�
<optimizer/gradients_2/optimizer/SquaredDifference_3_grad/SumSum>optimizer/gradients_2/optimizer/SquaredDifference_3_grad/mul_1Noptimizer/gradients_2/optimizer/SquaredDifference_3_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
@optimizer/gradients_2/optimizer/SquaredDifference_3_grad/ReshapeReshape<optimizer/gradients_2/optimizer/SquaredDifference_3_grad/Sum>optimizer/gradients_2/optimizer/SquaredDifference_3_grad/Shape*
T0*
Tshape0
�
>optimizer/gradients_2/optimizer/SquaredDifference_3_grad/Sum_1Sum>optimizer/gradients_2/optimizer/SquaredDifference_3_grad/mul_1Poptimizer/gradients_2/optimizer/SquaredDifference_3_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
Boptimizer/gradients_2/optimizer/SquaredDifference_3_grad/Reshape_1Reshape>optimizer/gradients_2/optimizer/SquaredDifference_3_grad/Sum_1@optimizer/gradients_2/optimizer/SquaredDifference_3_grad/Shape_1*
T0*
Tshape0
�
<optimizer/gradients_2/optimizer/SquaredDifference_3_grad/NegNegBoptimizer/gradients_2/optimizer/SquaredDifference_3_grad/Reshape_1*
T0
�
Ioptimizer/gradients_2/optimizer/SquaredDifference_3_grad/tuple/group_depsNoOp=^optimizer/gradients_2/optimizer/SquaredDifference_3_grad/NegA^optimizer/gradients_2/optimizer/SquaredDifference_3_grad/Reshape
�
Qoptimizer/gradients_2/optimizer/SquaredDifference_3_grad/tuple/control_dependencyIdentity@optimizer/gradients_2/optimizer/SquaredDifference_3_grad/ReshapeJ^optimizer/gradients_2/optimizer/SquaredDifference_3_grad/tuple/group_deps*
T0*S
_classI
GEloc:@optimizer/gradients_2/optimizer/SquaredDifference_3_grad/Reshape
�
Soptimizer/gradients_2/optimizer/SquaredDifference_3_grad/tuple/control_dependency_1Identity<optimizer/gradients_2/optimizer/SquaredDifference_3_grad/NegJ^optimizer/gradients_2/optimizer/SquaredDifference_3_grad/tuple/group_deps*
T0*O
_classE
CAloc:@optimizer/gradients_2/optimizer/SquaredDifference_3_grad/Neg
�
?optimizer/gradients_2/optimizer/SquaredDifference_4_grad/scalarConstJ^optimizer/gradients_2/optimizer/Maximum_1_grad/tuple/control_dependency_1*
dtype0*
valueB
 *   @
�
<optimizer/gradients_2/optimizer/SquaredDifference_4_grad/MulMul?optimizer/gradients_2/optimizer/SquaredDifference_4_grad/scalarIoptimizer/gradients_2/optimizer/Maximum_1_grad/tuple/control_dependency_1*
T0
�
<optimizer/gradients_2/optimizer/SquaredDifference_4_grad/subSuboptimizer/curiosity_returnsoptimizer/add_9J^optimizer/gradients_2/optimizer/Maximum_1_grad/tuple/control_dependency_1*
T0
�
>optimizer/gradients_2/optimizer/SquaredDifference_4_grad/mul_1Mul<optimizer/gradients_2/optimizer/SquaredDifference_4_grad/Mul<optimizer/gradients_2/optimizer/SquaredDifference_4_grad/sub*
T0
}
>optimizer/gradients_2/optimizer/SquaredDifference_4_grad/ShapeShapeoptimizer/curiosity_returns*
T0*
out_type0
s
@optimizer/gradients_2/optimizer/SquaredDifference_4_grad/Shape_1Shapeoptimizer/add_9*
T0*
out_type0
�
Noptimizer/gradients_2/optimizer/SquaredDifference_4_grad/BroadcastGradientArgsBroadcastGradientArgs>optimizer/gradients_2/optimizer/SquaredDifference_4_grad/Shape@optimizer/gradients_2/optimizer/SquaredDifference_4_grad/Shape_1*
T0
�
<optimizer/gradients_2/optimizer/SquaredDifference_4_grad/SumSum>optimizer/gradients_2/optimizer/SquaredDifference_4_grad/mul_1Noptimizer/gradients_2/optimizer/SquaredDifference_4_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
@optimizer/gradients_2/optimizer/SquaredDifference_4_grad/ReshapeReshape<optimizer/gradients_2/optimizer/SquaredDifference_4_grad/Sum>optimizer/gradients_2/optimizer/SquaredDifference_4_grad/Shape*
T0*
Tshape0
�
>optimizer/gradients_2/optimizer/SquaredDifference_4_grad/Sum_1Sum>optimizer/gradients_2/optimizer/SquaredDifference_4_grad/mul_1Poptimizer/gradients_2/optimizer/SquaredDifference_4_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
Boptimizer/gradients_2/optimizer/SquaredDifference_4_grad/Reshape_1Reshape>optimizer/gradients_2/optimizer/SquaredDifference_4_grad/Sum_1@optimizer/gradients_2/optimizer/SquaredDifference_4_grad/Shape_1*
T0*
Tshape0
�
<optimizer/gradients_2/optimizer/SquaredDifference_4_grad/NegNegBoptimizer/gradients_2/optimizer/SquaredDifference_4_grad/Reshape_1*
T0
�
Ioptimizer/gradients_2/optimizer/SquaredDifference_4_grad/tuple/group_depsNoOp=^optimizer/gradients_2/optimizer/SquaredDifference_4_grad/NegA^optimizer/gradients_2/optimizer/SquaredDifference_4_grad/Reshape
�
Qoptimizer/gradients_2/optimizer/SquaredDifference_4_grad/tuple/control_dependencyIdentity@optimizer/gradients_2/optimizer/SquaredDifference_4_grad/ReshapeJ^optimizer/gradients_2/optimizer/SquaredDifference_4_grad/tuple/group_deps*
T0*S
_classI
GEloc:@optimizer/gradients_2/optimizer/SquaredDifference_4_grad/Reshape
�
Soptimizer/gradients_2/optimizer/SquaredDifference_4_grad/tuple/control_dependency_1Identity<optimizer/gradients_2/optimizer/SquaredDifference_4_grad/NegJ^optimizer/gradients_2/optimizer/SquaredDifference_4_grad/tuple/group_deps*
T0*O
_classE
CAloc:@optimizer/gradients_2/optimizer/SquaredDifference_4_grad/Neg
�
Soptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits/Reshape_grad/ShapeShapepolicy_1/strided_slice_5*
T0*
out_type0
�
Uoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits/Reshape_grad/ReshapeReshape^optimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_grad/tuple/control_dependencySoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits/Reshape_grad/Shape*
T0*
Tshape0
�
Uoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits/Reshape_1_grad/ShapeShapepolicy_1/Softmax_2*
T0*
out_type0
�
Woptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits/Reshape_1_grad/ReshapeReshape`optimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_grad/tuple/control_dependency_1Uoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits/Reshape_1_grad/Shape*
T0*
Tshape0
�
Uoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_1/Reshape_grad/ShapeShapepolicy_1/strided_slice_7*
T0*
out_type0
�
Woptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_1/Reshape_grad/ReshapeReshape`optimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_1_grad/tuple/control_dependencyUoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_1/Reshape_grad/Shape*
T0*
Tshape0
�
Woptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_1/Reshape_1_grad/ShapeShapepolicy_1/Softmax_3*
T0*
out_type0
�
Yoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_1/Reshape_1_grad/ReshapeReshapeboptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_1_grad/tuple/control_dependency_1Woptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_1/Reshape_1_grad/Shape*
T0*
Tshape0
�
optimizer/gradients_2/AddNAddNCoptimizer/gradients_2/optimizer/mul_7_grad/tuple/control_dependencyUoptimizer/gradients_2/optimizer/clip_by_value_2/Minimum_grad/tuple/control_dependency*
N*
T0*E
_class;
97loc:@optimizer/gradients_2/optimizer/mul_7_grad/Reshape
g
,optimizer/gradients_2/optimizer/Exp_grad/mulMuloptimizer/gradients_2/AddNoptimizer/Exp*
T0
u
0optimizer/gradients_2/optimizer/Sum_6_grad/ShapeShape!optimizer/extrinsic_value/BiasAdd*
T0*
out_type0
�
/optimizer/gradients_2/optimizer/Sum_6_grad/SizeConst*C
_class9
75loc:@optimizer/gradients_2/optimizer/Sum_6_grad/Shape*
dtype0*
value	B :
�
.optimizer/gradients_2/optimizer/Sum_6_grad/addAddV2!optimizer/Sum_6/reduction_indices/optimizer/gradients_2/optimizer/Sum_6_grad/Size*
T0*C
_class9
75loc:@optimizer/gradients_2/optimizer/Sum_6_grad/Shape
�
.optimizer/gradients_2/optimizer/Sum_6_grad/modFloorMod.optimizer/gradients_2/optimizer/Sum_6_grad/add/optimizer/gradients_2/optimizer/Sum_6_grad/Size*
T0*C
_class9
75loc:@optimizer/gradients_2/optimizer/Sum_6_grad/Shape
�
2optimizer/gradients_2/optimizer/Sum_6_grad/Shape_1Const*C
_class9
75loc:@optimizer/gradients_2/optimizer/Sum_6_grad/Shape*
dtype0*
valueB 
�
6optimizer/gradients_2/optimizer/Sum_6_grad/range/startConst*C
_class9
75loc:@optimizer/gradients_2/optimizer/Sum_6_grad/Shape*
dtype0*
value	B : 
�
6optimizer/gradients_2/optimizer/Sum_6_grad/range/deltaConst*C
_class9
75loc:@optimizer/gradients_2/optimizer/Sum_6_grad/Shape*
dtype0*
value	B :
�
0optimizer/gradients_2/optimizer/Sum_6_grad/rangeRange6optimizer/gradients_2/optimizer/Sum_6_grad/range/start/optimizer/gradients_2/optimizer/Sum_6_grad/Size6optimizer/gradients_2/optimizer/Sum_6_grad/range/delta*

Tidx0*C
_class9
75loc:@optimizer/gradients_2/optimizer/Sum_6_grad/Shape
�
5optimizer/gradients_2/optimizer/Sum_6_grad/Fill/valueConst*C
_class9
75loc:@optimizer/gradients_2/optimizer/Sum_6_grad/Shape*
dtype0*
value	B :
�
/optimizer/gradients_2/optimizer/Sum_6_grad/FillFill2optimizer/gradients_2/optimizer/Sum_6_grad/Shape_15optimizer/gradients_2/optimizer/Sum_6_grad/Fill/value*
T0*C
_class9
75loc:@optimizer/gradients_2/optimizer/Sum_6_grad/Shape*

index_type0
�
8optimizer/gradients_2/optimizer/Sum_6_grad/DynamicStitchDynamicStitch0optimizer/gradients_2/optimizer/Sum_6_grad/range.optimizer/gradients_2/optimizer/Sum_6_grad/mod0optimizer/gradients_2/optimizer/Sum_6_grad/Shape/optimizer/gradients_2/optimizer/Sum_6_grad/Fill*
N*
T0*C
_class9
75loc:@optimizer/gradients_2/optimizer/Sum_6_grad/Shape
�
2optimizer/gradients_2/optimizer/Sum_6_grad/ReshapeReshapeSoptimizer/gradients_2/optimizer/SquaredDifference_1_grad/tuple/control_dependency_18optimizer/gradients_2/optimizer/Sum_6_grad/DynamicStitch*
T0*
Tshape0
�
6optimizer/gradients_2/optimizer/Sum_6_grad/BroadcastToBroadcastTo2optimizer/gradients_2/optimizer/Sum_6_grad/Reshape0optimizer/gradients_2/optimizer/Sum_6_grad/Shape*
T0*

Tidx0
v
0optimizer/gradients_2/optimizer/add_8_grad/ShapeShape"optimizer/extrinsic_value_estimate*
T0*
out_type0
m
2optimizer/gradients_2/optimizer/add_8_grad/Shape_1Shapeoptimizer/clip_by_value*
T0*
out_type0
�
@optimizer/gradients_2/optimizer/add_8_grad/BroadcastGradientArgsBroadcastGradientArgs0optimizer/gradients_2/optimizer/add_8_grad/Shape2optimizer/gradients_2/optimizer/add_8_grad/Shape_1*
T0
�
.optimizer/gradients_2/optimizer/add_8_grad/SumSumSoptimizer/gradients_2/optimizer/SquaredDifference_2_grad/tuple/control_dependency_1@optimizer/gradients_2/optimizer/add_8_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
2optimizer/gradients_2/optimizer/add_8_grad/ReshapeReshape.optimizer/gradients_2/optimizer/add_8_grad/Sum0optimizer/gradients_2/optimizer/add_8_grad/Shape*
T0*
Tshape0
�
0optimizer/gradients_2/optimizer/add_8_grad/Sum_1SumSoptimizer/gradients_2/optimizer/SquaredDifference_2_grad/tuple/control_dependency_1Boptimizer/gradients_2/optimizer/add_8_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
4optimizer/gradients_2/optimizer/add_8_grad/Reshape_1Reshape0optimizer/gradients_2/optimizer/add_8_grad/Sum_12optimizer/gradients_2/optimizer/add_8_grad/Shape_1*
T0*
Tshape0
�
;optimizer/gradients_2/optimizer/add_8_grad/tuple/group_depsNoOp3^optimizer/gradients_2/optimizer/add_8_grad/Reshape5^optimizer/gradients_2/optimizer/add_8_grad/Reshape_1
�
Coptimizer/gradients_2/optimizer/add_8_grad/tuple/control_dependencyIdentity2optimizer/gradients_2/optimizer/add_8_grad/Reshape<^optimizer/gradients_2/optimizer/add_8_grad/tuple/group_deps*
T0*E
_class;
97loc:@optimizer/gradients_2/optimizer/add_8_grad/Reshape
�
Eoptimizer/gradients_2/optimizer/add_8_grad/tuple/control_dependency_1Identity4optimizer/gradients_2/optimizer/add_8_grad/Reshape_1<^optimizer/gradients_2/optimizer/add_8_grad/tuple/group_deps*
T0*G
_class=
;9loc:@optimizer/gradients_2/optimizer/add_8_grad/Reshape_1
u
0optimizer/gradients_2/optimizer/Sum_8_grad/ShapeShape!optimizer/curiosity_value/BiasAdd*
T0*
out_type0
�
/optimizer/gradients_2/optimizer/Sum_8_grad/SizeConst*C
_class9
75loc:@optimizer/gradients_2/optimizer/Sum_8_grad/Shape*
dtype0*
value	B :
�
.optimizer/gradients_2/optimizer/Sum_8_grad/addAddV2!optimizer/Sum_8/reduction_indices/optimizer/gradients_2/optimizer/Sum_8_grad/Size*
T0*C
_class9
75loc:@optimizer/gradients_2/optimizer/Sum_8_grad/Shape
�
.optimizer/gradients_2/optimizer/Sum_8_grad/modFloorMod.optimizer/gradients_2/optimizer/Sum_8_grad/add/optimizer/gradients_2/optimizer/Sum_8_grad/Size*
T0*C
_class9
75loc:@optimizer/gradients_2/optimizer/Sum_8_grad/Shape
�
2optimizer/gradients_2/optimizer/Sum_8_grad/Shape_1Const*C
_class9
75loc:@optimizer/gradients_2/optimizer/Sum_8_grad/Shape*
dtype0*
valueB 
�
6optimizer/gradients_2/optimizer/Sum_8_grad/range/startConst*C
_class9
75loc:@optimizer/gradients_2/optimizer/Sum_8_grad/Shape*
dtype0*
value	B : 
�
6optimizer/gradients_2/optimizer/Sum_8_grad/range/deltaConst*C
_class9
75loc:@optimizer/gradients_2/optimizer/Sum_8_grad/Shape*
dtype0*
value	B :
�
0optimizer/gradients_2/optimizer/Sum_8_grad/rangeRange6optimizer/gradients_2/optimizer/Sum_8_grad/range/start/optimizer/gradients_2/optimizer/Sum_8_grad/Size6optimizer/gradients_2/optimizer/Sum_8_grad/range/delta*

Tidx0*C
_class9
75loc:@optimizer/gradients_2/optimizer/Sum_8_grad/Shape
�
5optimizer/gradients_2/optimizer/Sum_8_grad/Fill/valueConst*C
_class9
75loc:@optimizer/gradients_2/optimizer/Sum_8_grad/Shape*
dtype0*
value	B :
�
/optimizer/gradients_2/optimizer/Sum_8_grad/FillFill2optimizer/gradients_2/optimizer/Sum_8_grad/Shape_15optimizer/gradients_2/optimizer/Sum_8_grad/Fill/value*
T0*C
_class9
75loc:@optimizer/gradients_2/optimizer/Sum_8_grad/Shape*

index_type0
�
8optimizer/gradients_2/optimizer/Sum_8_grad/DynamicStitchDynamicStitch0optimizer/gradients_2/optimizer/Sum_8_grad/range.optimizer/gradients_2/optimizer/Sum_8_grad/mod0optimizer/gradients_2/optimizer/Sum_8_grad/Shape/optimizer/gradients_2/optimizer/Sum_8_grad/Fill*
N*
T0*C
_class9
75loc:@optimizer/gradients_2/optimizer/Sum_8_grad/Shape
�
2optimizer/gradients_2/optimizer/Sum_8_grad/ReshapeReshapeSoptimizer/gradients_2/optimizer/SquaredDifference_3_grad/tuple/control_dependency_18optimizer/gradients_2/optimizer/Sum_8_grad/DynamicStitch*
T0*
Tshape0
�
6optimizer/gradients_2/optimizer/Sum_8_grad/BroadcastToBroadcastTo2optimizer/gradients_2/optimizer/Sum_8_grad/Reshape0optimizer/gradients_2/optimizer/Sum_8_grad/Shape*
T0*

Tidx0
v
0optimizer/gradients_2/optimizer/add_9_grad/ShapeShape"optimizer/curiosity_value_estimate*
T0*
out_type0
o
2optimizer/gradients_2/optimizer/add_9_grad/Shape_1Shapeoptimizer/clip_by_value_1*
T0*
out_type0
�
@optimizer/gradients_2/optimizer/add_9_grad/BroadcastGradientArgsBroadcastGradientArgs0optimizer/gradients_2/optimizer/add_9_grad/Shape2optimizer/gradients_2/optimizer/add_9_grad/Shape_1*
T0
�
.optimizer/gradients_2/optimizer/add_9_grad/SumSumSoptimizer/gradients_2/optimizer/SquaredDifference_4_grad/tuple/control_dependency_1@optimizer/gradients_2/optimizer/add_9_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
2optimizer/gradients_2/optimizer/add_9_grad/ReshapeReshape.optimizer/gradients_2/optimizer/add_9_grad/Sum0optimizer/gradients_2/optimizer/add_9_grad/Shape*
T0*
Tshape0
�
0optimizer/gradients_2/optimizer/add_9_grad/Sum_1SumSoptimizer/gradients_2/optimizer/SquaredDifference_4_grad/tuple/control_dependency_1Boptimizer/gradients_2/optimizer/add_9_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
4optimizer/gradients_2/optimizer/add_9_grad/Reshape_1Reshape0optimizer/gradients_2/optimizer/add_9_grad/Sum_12optimizer/gradients_2/optimizer/add_9_grad/Shape_1*
T0*
Tshape0
�
;optimizer/gradients_2/optimizer/add_9_grad/tuple/group_depsNoOp3^optimizer/gradients_2/optimizer/add_9_grad/Reshape5^optimizer/gradients_2/optimizer/add_9_grad/Reshape_1
�
Coptimizer/gradients_2/optimizer/add_9_grad/tuple/control_dependencyIdentity2optimizer/gradients_2/optimizer/add_9_grad/Reshape<^optimizer/gradients_2/optimizer/add_9_grad/tuple/group_deps*
T0*E
_class;
97loc:@optimizer/gradients_2/optimizer/add_9_grad/Reshape
�
Eoptimizer/gradients_2/optimizer/add_9_grad/tuple/control_dependency_1Identity4optimizer/gradients_2/optimizer/add_9_grad/Reshape_1<^optimizer/gradients_2/optimizer/add_9_grad/tuple/group_deps*
T0*G
_class=
;9loc:@optimizer/gradients_2/optimizer/add_9_grad/Reshape_1
n
9optimizer/gradients_2/policy_1/strided_slice_5_grad/ShapeShapepolicy_1/concat_2*
T0*
out_type0

Joptimizer/gradients_2/policy_1/strided_slice_5_grad/StridedSliceGrad/beginConst*
dtype0*
valueB"        
}
Hoptimizer/gradients_2/policy_1/strided_slice_5_grad/StridedSliceGrad/endConst*
dtype0*
valueB"       
�
Loptimizer/gradients_2/policy_1/strided_slice_5_grad/StridedSliceGrad/stridesConst*
dtype0*
valueB"      
�
Doptimizer/gradients_2/policy_1/strided_slice_5_grad/StridedSliceGradStridedSliceGrad9optimizer/gradients_2/policy_1/strided_slice_5_grad/ShapeJoptimizer/gradients_2/policy_1/strided_slice_5_grad/StridedSliceGrad/beginHoptimizer/gradients_2/policy_1/strided_slice_5_grad/StridedSliceGrad/endLoptimizer/gradients_2/policy_1/strided_slice_5_grad/StridedSliceGrad/stridesUoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits/Reshape_grad/Reshape*
Index0*
T0*

begin_mask*
ellipsis_mask *
end_mask*
new_axis_mask *
shrink_axis_mask 
�
1optimizer/gradients_2/policy_1/Softmax_2_grad/mulMulWoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits/Reshape_1_grad/Reshapepolicy_1/Softmax_2*
T0
v
Coptimizer/gradients_2/policy_1/Softmax_2_grad/Sum/reduction_indicesConst*
dtype0*
valueB :
���������
�
1optimizer/gradients_2/policy_1/Softmax_2_grad/SumSum1optimizer/gradients_2/policy_1/Softmax_2_grad/mulCoptimizer/gradients_2/policy_1/Softmax_2_grad/Sum/reduction_indices*
T0*

Tidx0*
	keep_dims(
�
1optimizer/gradients_2/policy_1/Softmax_2_grad/subSubWoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits/Reshape_1_grad/Reshape1optimizer/gradients_2/policy_1/Softmax_2_grad/Sum*
T0
�
3optimizer/gradients_2/policy_1/Softmax_2_grad/mul_1Mul1optimizer/gradients_2/policy_1/Softmax_2_grad/subpolicy_1/Softmax_2*
T0
n
9optimizer/gradients_2/policy_1/strided_slice_7_grad/ShapeShapepolicy_1/concat_2*
T0*
out_type0

Joptimizer/gradients_2/policy_1/strided_slice_7_grad/StridedSliceGrad/beginConst*
dtype0*
valueB"       
}
Hoptimizer/gradients_2/policy_1/strided_slice_7_grad/StridedSliceGrad/endConst*
dtype0*
valueB"       
�
Loptimizer/gradients_2/policy_1/strided_slice_7_grad/StridedSliceGrad/stridesConst*
dtype0*
valueB"      
�
Doptimizer/gradients_2/policy_1/strided_slice_7_grad/StridedSliceGradStridedSliceGrad9optimizer/gradients_2/policy_1/strided_slice_7_grad/ShapeJoptimizer/gradients_2/policy_1/strided_slice_7_grad/StridedSliceGrad/beginHoptimizer/gradients_2/policy_1/strided_slice_7_grad/StridedSliceGrad/endLoptimizer/gradients_2/policy_1/strided_slice_7_grad/StridedSliceGrad/stridesWoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_1/Reshape_grad/Reshape*
Index0*
T0*

begin_mask*
ellipsis_mask *
end_mask*
new_axis_mask *
shrink_axis_mask 
�
1optimizer/gradients_2/policy_1/Softmax_3_grad/mulMulYoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_1/Reshape_1_grad/Reshapepolicy_1/Softmax_3*
T0
v
Coptimizer/gradients_2/policy_1/Softmax_3_grad/Sum/reduction_indicesConst*
dtype0*
valueB :
���������
�
1optimizer/gradients_2/policy_1/Softmax_3_grad/SumSum1optimizer/gradients_2/policy_1/Softmax_3_grad/mulCoptimizer/gradients_2/policy_1/Softmax_3_grad/Sum/reduction_indices*
T0*

Tidx0*
	keep_dims(
�
1optimizer/gradients_2/policy_1/Softmax_3_grad/subSubYoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_1/Reshape_1_grad/Reshape1optimizer/gradients_2/policy_1/Softmax_3_grad/Sum*
T0
�
3optimizer/gradients_2/policy_1/Softmax_3_grad/mul_1Mul1optimizer/gradients_2/policy_1/Softmax_3_grad/subpolicy_1/Softmax_3*
T0
b
0optimizer/gradients_2/optimizer/sub_2_grad/ShapeShapepolicy_1/Sum_3*
T0*
out_type0
e
2optimizer/gradients_2/optimizer/sub_2_grad/Shape_1Shapeoptimizer/Sum_4*
T0*
out_type0
�
@optimizer/gradients_2/optimizer/sub_2_grad/BroadcastGradientArgsBroadcastGradientArgs0optimizer/gradients_2/optimizer/sub_2_grad/Shape2optimizer/gradients_2/optimizer/sub_2_grad/Shape_1*
T0
�
.optimizer/gradients_2/optimizer/sub_2_grad/SumSum,optimizer/gradients_2/optimizer/Exp_grad/mul@optimizer/gradients_2/optimizer/sub_2_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
2optimizer/gradients_2/optimizer/sub_2_grad/ReshapeReshape.optimizer/gradients_2/optimizer/sub_2_grad/Sum0optimizer/gradients_2/optimizer/sub_2_grad/Shape*
T0*
Tshape0
l
.optimizer/gradients_2/optimizer/sub_2_grad/NegNeg,optimizer/gradients_2/optimizer/Exp_grad/mul*
T0
�
0optimizer/gradients_2/optimizer/sub_2_grad/Sum_1Sum.optimizer/gradients_2/optimizer/sub_2_grad/NegBoptimizer/gradients_2/optimizer/sub_2_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
4optimizer/gradients_2/optimizer/sub_2_grad/Reshape_1Reshape0optimizer/gradients_2/optimizer/sub_2_grad/Sum_12optimizer/gradients_2/optimizer/sub_2_grad/Shape_1*
T0*
Tshape0
�
;optimizer/gradients_2/optimizer/sub_2_grad/tuple/group_depsNoOp3^optimizer/gradients_2/optimizer/sub_2_grad/Reshape5^optimizer/gradients_2/optimizer/sub_2_grad/Reshape_1
�
Coptimizer/gradients_2/optimizer/sub_2_grad/tuple/control_dependencyIdentity2optimizer/gradients_2/optimizer/sub_2_grad/Reshape<^optimizer/gradients_2/optimizer/sub_2_grad/tuple/group_deps*
T0*E
_class;
97loc:@optimizer/gradients_2/optimizer/sub_2_grad/Reshape
�
Eoptimizer/gradients_2/optimizer/sub_2_grad/tuple/control_dependency_1Identity4optimizer/gradients_2/optimizer/sub_2_grad/Reshape_1<^optimizer/gradients_2/optimizer/sub_2_grad/tuple/group_deps*
T0*G
_class=
;9loc:@optimizer/gradients_2/optimizer/sub_2_grad/Reshape_1
{
8optimizer/gradients_2/optimizer/clip_by_value_grad/ShapeShapeoptimizer/clip_by_value/Minimum*
T0*
out_type0
c
:optimizer/gradients_2/optimizer/clip_by_value_grad/Shape_1Const*
dtype0*
valueB 
�
:optimizer/gradients_2/optimizer/clip_by_value_grad/Shape_2ShapeEoptimizer/gradients_2/optimizer/add_8_grad/tuple/control_dependency_1*
T0*
out_type0
k
>optimizer/gradients_2/optimizer/clip_by_value_grad/zeros/ConstConst*
dtype0*
valueB
 *    
�
8optimizer/gradients_2/optimizer/clip_by_value_grad/zerosFill:optimizer/gradients_2/optimizer/clip_by_value_grad/Shape_2>optimizer/gradients_2/optimizer/clip_by_value_grad/zeros/Const*
T0*

index_type0
�
?optimizer/gradients_2/optimizer/clip_by_value_grad/GreaterEqualGreaterEqualoptimizer/clip_by_value/Minimumoptimizer/Neg_3*
T0
�
Hoptimizer/gradients_2/optimizer/clip_by_value_grad/BroadcastGradientArgsBroadcastGradientArgs8optimizer/gradients_2/optimizer/clip_by_value_grad/Shape:optimizer/gradients_2/optimizer/clip_by_value_grad/Shape_1*
T0
�
;optimizer/gradients_2/optimizer/clip_by_value_grad/SelectV2SelectV2?optimizer/gradients_2/optimizer/clip_by_value_grad/GreaterEqualEoptimizer/gradients_2/optimizer/add_8_grad/tuple/control_dependency_18optimizer/gradients_2/optimizer/clip_by_value_grad/zeros*
T0
�
6optimizer/gradients_2/optimizer/clip_by_value_grad/SumSum;optimizer/gradients_2/optimizer/clip_by_value_grad/SelectV2Hoptimizer/gradients_2/optimizer/clip_by_value_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
:optimizer/gradients_2/optimizer/clip_by_value_grad/ReshapeReshape6optimizer/gradients_2/optimizer/clip_by_value_grad/Sum8optimizer/gradients_2/optimizer/clip_by_value_grad/Shape*
T0*
Tshape0
�
=optimizer/gradients_2/optimizer/clip_by_value_grad/SelectV2_1SelectV2?optimizer/gradients_2/optimizer/clip_by_value_grad/GreaterEqual8optimizer/gradients_2/optimizer/clip_by_value_grad/zerosEoptimizer/gradients_2/optimizer/add_8_grad/tuple/control_dependency_1*
T0
�
8optimizer/gradients_2/optimizer/clip_by_value_grad/Sum_1Sum=optimizer/gradients_2/optimizer/clip_by_value_grad/SelectV2_1Joptimizer/gradients_2/optimizer/clip_by_value_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
<optimizer/gradients_2/optimizer/clip_by_value_grad/Reshape_1Reshape8optimizer/gradients_2/optimizer/clip_by_value_grad/Sum_1:optimizer/gradients_2/optimizer/clip_by_value_grad/Shape_1*
T0*
Tshape0
�
Coptimizer/gradients_2/optimizer/clip_by_value_grad/tuple/group_depsNoOp;^optimizer/gradients_2/optimizer/clip_by_value_grad/Reshape=^optimizer/gradients_2/optimizer/clip_by_value_grad/Reshape_1
�
Koptimizer/gradients_2/optimizer/clip_by_value_grad/tuple/control_dependencyIdentity:optimizer/gradients_2/optimizer/clip_by_value_grad/ReshapeD^optimizer/gradients_2/optimizer/clip_by_value_grad/tuple/group_deps*
T0*M
_classC
A?loc:@optimizer/gradients_2/optimizer/clip_by_value_grad/Reshape
�
Moptimizer/gradients_2/optimizer/clip_by_value_grad/tuple/control_dependency_1Identity<optimizer/gradients_2/optimizer/clip_by_value_grad/Reshape_1D^optimizer/gradients_2/optimizer/clip_by_value_grad/tuple/group_deps*
T0*O
_classE
CAloc:@optimizer/gradients_2/optimizer/clip_by_value_grad/Reshape_1

:optimizer/gradients_2/optimizer/clip_by_value_1_grad/ShapeShape!optimizer/clip_by_value_1/Minimum*
T0*
out_type0
e
<optimizer/gradients_2/optimizer/clip_by_value_1_grad/Shape_1Const*
dtype0*
valueB 
�
<optimizer/gradients_2/optimizer/clip_by_value_1_grad/Shape_2ShapeEoptimizer/gradients_2/optimizer/add_9_grad/tuple/control_dependency_1*
T0*
out_type0
m
@optimizer/gradients_2/optimizer/clip_by_value_1_grad/zeros/ConstConst*
dtype0*
valueB
 *    
�
:optimizer/gradients_2/optimizer/clip_by_value_1_grad/zerosFill<optimizer/gradients_2/optimizer/clip_by_value_1_grad/Shape_2@optimizer/gradients_2/optimizer/clip_by_value_1_grad/zeros/Const*
T0*

index_type0
�
Aoptimizer/gradients_2/optimizer/clip_by_value_1_grad/GreaterEqualGreaterEqual!optimizer/clip_by_value_1/Minimumoptimizer/Neg_4*
T0
�
Joptimizer/gradients_2/optimizer/clip_by_value_1_grad/BroadcastGradientArgsBroadcastGradientArgs:optimizer/gradients_2/optimizer/clip_by_value_1_grad/Shape<optimizer/gradients_2/optimizer/clip_by_value_1_grad/Shape_1*
T0
�
=optimizer/gradients_2/optimizer/clip_by_value_1_grad/SelectV2SelectV2Aoptimizer/gradients_2/optimizer/clip_by_value_1_grad/GreaterEqualEoptimizer/gradients_2/optimizer/add_9_grad/tuple/control_dependency_1:optimizer/gradients_2/optimizer/clip_by_value_1_grad/zeros*
T0
�
8optimizer/gradients_2/optimizer/clip_by_value_1_grad/SumSum=optimizer/gradients_2/optimizer/clip_by_value_1_grad/SelectV2Joptimizer/gradients_2/optimizer/clip_by_value_1_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
<optimizer/gradients_2/optimizer/clip_by_value_1_grad/ReshapeReshape8optimizer/gradients_2/optimizer/clip_by_value_1_grad/Sum:optimizer/gradients_2/optimizer/clip_by_value_1_grad/Shape*
T0*
Tshape0
�
?optimizer/gradients_2/optimizer/clip_by_value_1_grad/SelectV2_1SelectV2Aoptimizer/gradients_2/optimizer/clip_by_value_1_grad/GreaterEqual:optimizer/gradients_2/optimizer/clip_by_value_1_grad/zerosEoptimizer/gradients_2/optimizer/add_9_grad/tuple/control_dependency_1*
T0
�
:optimizer/gradients_2/optimizer/clip_by_value_1_grad/Sum_1Sum?optimizer/gradients_2/optimizer/clip_by_value_1_grad/SelectV2_1Loptimizer/gradients_2/optimizer/clip_by_value_1_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
>optimizer/gradients_2/optimizer/clip_by_value_1_grad/Reshape_1Reshape:optimizer/gradients_2/optimizer/clip_by_value_1_grad/Sum_1<optimizer/gradients_2/optimizer/clip_by_value_1_grad/Shape_1*
T0*
Tshape0
�
Eoptimizer/gradients_2/optimizer/clip_by_value_1_grad/tuple/group_depsNoOp=^optimizer/gradients_2/optimizer/clip_by_value_1_grad/Reshape?^optimizer/gradients_2/optimizer/clip_by_value_1_grad/Reshape_1
�
Moptimizer/gradients_2/optimizer/clip_by_value_1_grad/tuple/control_dependencyIdentity<optimizer/gradients_2/optimizer/clip_by_value_1_grad/ReshapeF^optimizer/gradients_2/optimizer/clip_by_value_1_grad/tuple/group_deps*
T0*O
_classE
CAloc:@optimizer/gradients_2/optimizer/clip_by_value_1_grad/Reshape
�
Ooptimizer/gradients_2/optimizer/clip_by_value_1_grad/tuple/control_dependency_1Identity>optimizer/gradients_2/optimizer/clip_by_value_1_grad/Reshape_1F^optimizer/gradients_2/optimizer/clip_by_value_1_grad/tuple/group_deps*
T0*Q
_classG
ECloc:@optimizer/gradients_2/optimizer/clip_by_value_1_grad/Reshape_1
n
9optimizer/gradients_2/policy_1/strided_slice_4_grad/ShapeShapepolicy_1/concat_2*
T0*
out_type0

Joptimizer/gradients_2/policy_1/strided_slice_4_grad/StridedSliceGrad/beginConst*
dtype0*
valueB"        
}
Hoptimizer/gradients_2/policy_1/strided_slice_4_grad/StridedSliceGrad/endConst*
dtype0*
valueB"       
�
Loptimizer/gradients_2/policy_1/strided_slice_4_grad/StridedSliceGrad/stridesConst*
dtype0*
valueB"      
�
Doptimizer/gradients_2/policy_1/strided_slice_4_grad/StridedSliceGradStridedSliceGrad9optimizer/gradients_2/policy_1/strided_slice_4_grad/ShapeJoptimizer/gradients_2/policy_1/strided_slice_4_grad/StridedSliceGrad/beginHoptimizer/gradients_2/policy_1/strided_slice_4_grad/StridedSliceGrad/endLoptimizer/gradients_2/policy_1/strided_slice_4_grad/StridedSliceGrad/strides3optimizer/gradients_2/policy_1/Softmax_2_grad/mul_1*
Index0*
T0*

begin_mask*
ellipsis_mask *
end_mask*
new_axis_mask *
shrink_axis_mask 
n
9optimizer/gradients_2/policy_1/strided_slice_6_grad/ShapeShapepolicy_1/concat_2*
T0*
out_type0

Joptimizer/gradients_2/policy_1/strided_slice_6_grad/StridedSliceGrad/beginConst*
dtype0*
valueB"       
}
Hoptimizer/gradients_2/policy_1/strided_slice_6_grad/StridedSliceGrad/endConst*
dtype0*
valueB"       
�
Loptimizer/gradients_2/policy_1/strided_slice_6_grad/StridedSliceGrad/stridesConst*
dtype0*
valueB"      
�
Doptimizer/gradients_2/policy_1/strided_slice_6_grad/StridedSliceGradStridedSliceGrad9optimizer/gradients_2/policy_1/strided_slice_6_grad/ShapeJoptimizer/gradients_2/policy_1/strided_slice_6_grad/StridedSliceGrad/beginHoptimizer/gradients_2/policy_1/strided_slice_6_grad/StridedSliceGrad/endLoptimizer/gradients_2/policy_1/strided_slice_6_grad/StridedSliceGrad/strides3optimizer/gradients_2/policy_1/Softmax_3_grad/mul_1*
Index0*
T0*

begin_mask*
ellipsis_mask *
end_mask*
new_axis_mask *
shrink_axis_mask 
c
/optimizer/gradients_2/policy_1/Sum_3_grad/ShapeShapepolicy_1/stack_1*
T0*
out_type0
�
5optimizer/gradients_2/policy_1/Sum_3_grad/BroadcastToBroadcastToCoptimizer/gradients_2/optimizer/sub_2_grad/tuple/control_dependency/optimizer/gradients_2/policy_1/Sum_3_grad/Shape*
T0*

Tidx0
q
@optimizer/gradients_2/optimizer/clip_by_value/Minimum_grad/ShapeShapeoptimizer/sub*
T0*
out_type0
k
Boptimizer/gradients_2/optimizer/clip_by_value/Minimum_grad/Shape_1Const*
dtype0*
valueB 
�
Boptimizer/gradients_2/optimizer/clip_by_value/Minimum_grad/Shape_2ShapeKoptimizer/gradients_2/optimizer/clip_by_value_grad/tuple/control_dependency*
T0*
out_type0
s
Foptimizer/gradients_2/optimizer/clip_by_value/Minimum_grad/zeros/ConstConst*
dtype0*
valueB
 *    
�
@optimizer/gradients_2/optimizer/clip_by_value/Minimum_grad/zerosFillBoptimizer/gradients_2/optimizer/clip_by_value/Minimum_grad/Shape_2Foptimizer/gradients_2/optimizer/clip_by_value/Minimum_grad/zeros/Const*
T0*

index_type0
�
Doptimizer/gradients_2/optimizer/clip_by_value/Minimum_grad/LessEqual	LessEqualoptimizer/suboptimizer/Variable_1/read*
T0
�
Poptimizer/gradients_2/optimizer/clip_by_value/Minimum_grad/BroadcastGradientArgsBroadcastGradientArgs@optimizer/gradients_2/optimizer/clip_by_value/Minimum_grad/ShapeBoptimizer/gradients_2/optimizer/clip_by_value/Minimum_grad/Shape_1*
T0
�
Coptimizer/gradients_2/optimizer/clip_by_value/Minimum_grad/SelectV2SelectV2Doptimizer/gradients_2/optimizer/clip_by_value/Minimum_grad/LessEqualKoptimizer/gradients_2/optimizer/clip_by_value_grad/tuple/control_dependency@optimizer/gradients_2/optimizer/clip_by_value/Minimum_grad/zeros*
T0
�
>optimizer/gradients_2/optimizer/clip_by_value/Minimum_grad/SumSumCoptimizer/gradients_2/optimizer/clip_by_value/Minimum_grad/SelectV2Poptimizer/gradients_2/optimizer/clip_by_value/Minimum_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
Boptimizer/gradients_2/optimizer/clip_by_value/Minimum_grad/ReshapeReshape>optimizer/gradients_2/optimizer/clip_by_value/Minimum_grad/Sum@optimizer/gradients_2/optimizer/clip_by_value/Minimum_grad/Shape*
T0*
Tshape0
�
Eoptimizer/gradients_2/optimizer/clip_by_value/Minimum_grad/SelectV2_1SelectV2Doptimizer/gradients_2/optimizer/clip_by_value/Minimum_grad/LessEqual@optimizer/gradients_2/optimizer/clip_by_value/Minimum_grad/zerosKoptimizer/gradients_2/optimizer/clip_by_value_grad/tuple/control_dependency*
T0
�
@optimizer/gradients_2/optimizer/clip_by_value/Minimum_grad/Sum_1SumEoptimizer/gradients_2/optimizer/clip_by_value/Minimum_grad/SelectV2_1Roptimizer/gradients_2/optimizer/clip_by_value/Minimum_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
Doptimizer/gradients_2/optimizer/clip_by_value/Minimum_grad/Reshape_1Reshape@optimizer/gradients_2/optimizer/clip_by_value/Minimum_grad/Sum_1Boptimizer/gradients_2/optimizer/clip_by_value/Minimum_grad/Shape_1*
T0*
Tshape0
�
Koptimizer/gradients_2/optimizer/clip_by_value/Minimum_grad/tuple/group_depsNoOpC^optimizer/gradients_2/optimizer/clip_by_value/Minimum_grad/ReshapeE^optimizer/gradients_2/optimizer/clip_by_value/Minimum_grad/Reshape_1
�
Soptimizer/gradients_2/optimizer/clip_by_value/Minimum_grad/tuple/control_dependencyIdentityBoptimizer/gradients_2/optimizer/clip_by_value/Minimum_grad/ReshapeL^optimizer/gradients_2/optimizer/clip_by_value/Minimum_grad/tuple/group_deps*
T0*U
_classK
IGloc:@optimizer/gradients_2/optimizer/clip_by_value/Minimum_grad/Reshape
�
Uoptimizer/gradients_2/optimizer/clip_by_value/Minimum_grad/tuple/control_dependency_1IdentityDoptimizer/gradients_2/optimizer/clip_by_value/Minimum_grad/Reshape_1L^optimizer/gradients_2/optimizer/clip_by_value/Minimum_grad/tuple/group_deps*
T0*W
_classM
KIloc:@optimizer/gradients_2/optimizer/clip_by_value/Minimum_grad/Reshape_1
u
Boptimizer/gradients_2/optimizer/clip_by_value_1/Minimum_grad/ShapeShapeoptimizer/sub_1*
T0*
out_type0
m
Doptimizer/gradients_2/optimizer/clip_by_value_1/Minimum_grad/Shape_1Const*
dtype0*
valueB 
�
Doptimizer/gradients_2/optimizer/clip_by_value_1/Minimum_grad/Shape_2ShapeMoptimizer/gradients_2/optimizer/clip_by_value_1_grad/tuple/control_dependency*
T0*
out_type0
u
Hoptimizer/gradients_2/optimizer/clip_by_value_1/Minimum_grad/zeros/ConstConst*
dtype0*
valueB
 *    
�
Boptimizer/gradients_2/optimizer/clip_by_value_1/Minimum_grad/zerosFillDoptimizer/gradients_2/optimizer/clip_by_value_1/Minimum_grad/Shape_2Hoptimizer/gradients_2/optimizer/clip_by_value_1/Minimum_grad/zeros/Const*
T0*

index_type0
�
Foptimizer/gradients_2/optimizer/clip_by_value_1/Minimum_grad/LessEqual	LessEqualoptimizer/sub_1optimizer/Variable_1/read*
T0
�
Roptimizer/gradients_2/optimizer/clip_by_value_1/Minimum_grad/BroadcastGradientArgsBroadcastGradientArgsBoptimizer/gradients_2/optimizer/clip_by_value_1/Minimum_grad/ShapeDoptimizer/gradients_2/optimizer/clip_by_value_1/Minimum_grad/Shape_1*
T0
�
Eoptimizer/gradients_2/optimizer/clip_by_value_1/Minimum_grad/SelectV2SelectV2Foptimizer/gradients_2/optimizer/clip_by_value_1/Minimum_grad/LessEqualMoptimizer/gradients_2/optimizer/clip_by_value_1_grad/tuple/control_dependencyBoptimizer/gradients_2/optimizer/clip_by_value_1/Minimum_grad/zeros*
T0
�
@optimizer/gradients_2/optimizer/clip_by_value_1/Minimum_grad/SumSumEoptimizer/gradients_2/optimizer/clip_by_value_1/Minimum_grad/SelectV2Roptimizer/gradients_2/optimizer/clip_by_value_1/Minimum_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
Doptimizer/gradients_2/optimizer/clip_by_value_1/Minimum_grad/ReshapeReshape@optimizer/gradients_2/optimizer/clip_by_value_1/Minimum_grad/SumBoptimizer/gradients_2/optimizer/clip_by_value_1/Minimum_grad/Shape*
T0*
Tshape0
�
Goptimizer/gradients_2/optimizer/clip_by_value_1/Minimum_grad/SelectV2_1SelectV2Foptimizer/gradients_2/optimizer/clip_by_value_1/Minimum_grad/LessEqualBoptimizer/gradients_2/optimizer/clip_by_value_1/Minimum_grad/zerosMoptimizer/gradients_2/optimizer/clip_by_value_1_grad/tuple/control_dependency*
T0
�
Boptimizer/gradients_2/optimizer/clip_by_value_1/Minimum_grad/Sum_1SumGoptimizer/gradients_2/optimizer/clip_by_value_1/Minimum_grad/SelectV2_1Toptimizer/gradients_2/optimizer/clip_by_value_1/Minimum_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
Foptimizer/gradients_2/optimizer/clip_by_value_1/Minimum_grad/Reshape_1ReshapeBoptimizer/gradients_2/optimizer/clip_by_value_1/Minimum_grad/Sum_1Doptimizer/gradients_2/optimizer/clip_by_value_1/Minimum_grad/Shape_1*
T0*
Tshape0
�
Moptimizer/gradients_2/optimizer/clip_by_value_1/Minimum_grad/tuple/group_depsNoOpE^optimizer/gradients_2/optimizer/clip_by_value_1/Minimum_grad/ReshapeG^optimizer/gradients_2/optimizer/clip_by_value_1/Minimum_grad/Reshape_1
�
Uoptimizer/gradients_2/optimizer/clip_by_value_1/Minimum_grad/tuple/control_dependencyIdentityDoptimizer/gradients_2/optimizer/clip_by_value_1/Minimum_grad/ReshapeN^optimizer/gradients_2/optimizer/clip_by_value_1/Minimum_grad/tuple/group_deps*
T0*W
_classM
KIloc:@optimizer/gradients_2/optimizer/clip_by_value_1/Minimum_grad/Reshape
�
Woptimizer/gradients_2/optimizer/clip_by_value_1/Minimum_grad/tuple/control_dependency_1IdentityFoptimizer/gradients_2/optimizer/clip_by_value_1/Minimum_grad/Reshape_1N^optimizer/gradients_2/optimizer/clip_by_value_1/Minimum_grad/tuple/group_deps*
T0*Y
_classO
MKloc:@optimizer/gradients_2/optimizer/clip_by_value_1/Minimum_grad/Reshape_1
�
3optimizer/gradients_2/policy_1/stack_1_grad/unstackUnpack5optimizer/gradients_2/policy_1/Sum_3_grad/BroadcastTo*
T0*

axis*	
num
z
<optimizer/gradients_2/policy_1/stack_1_grad/tuple/group_depsNoOp4^optimizer/gradients_2/policy_1/stack_1_grad/unstack
�
Doptimizer/gradients_2/policy_1/stack_1_grad/tuple/control_dependencyIdentity3optimizer/gradients_2/policy_1/stack_1_grad/unstack=^optimizer/gradients_2/policy_1/stack_1_grad/tuple/group_deps*
T0*F
_class<
:8loc:@optimizer/gradients_2/policy_1/stack_1_grad/unstack
�
Foptimizer/gradients_2/policy_1/stack_1_grad/tuple/control_dependency_1Identity5optimizer/gradients_2/policy_1/stack_1_grad/unstack:1=^optimizer/gradients_2/policy_1/stack_1_grad/tuple/group_deps*
T0*F
_class<
:8loc:@optimizer/gradients_2/policy_1/stack_1_grad/unstack
a
.optimizer/gradients_2/optimizer/sub_grad/ShapeShapeoptimizer/Sum_5*
T0*
out_type0
v
0optimizer/gradients_2/optimizer/sub_grad/Shape_1Shape"optimizer/extrinsic_value_estimate*
T0*
out_type0
�
>optimizer/gradients_2/optimizer/sub_grad/BroadcastGradientArgsBroadcastGradientArgs.optimizer/gradients_2/optimizer/sub_grad/Shape0optimizer/gradients_2/optimizer/sub_grad/Shape_1*
T0
�
,optimizer/gradients_2/optimizer/sub_grad/SumSumSoptimizer/gradients_2/optimizer/clip_by_value/Minimum_grad/tuple/control_dependency>optimizer/gradients_2/optimizer/sub_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
0optimizer/gradients_2/optimizer/sub_grad/ReshapeReshape,optimizer/gradients_2/optimizer/sub_grad/Sum.optimizer/gradients_2/optimizer/sub_grad/Shape*
T0*
Tshape0
�
,optimizer/gradients_2/optimizer/sub_grad/NegNegSoptimizer/gradients_2/optimizer/clip_by_value/Minimum_grad/tuple/control_dependency*
T0
�
.optimizer/gradients_2/optimizer/sub_grad/Sum_1Sum,optimizer/gradients_2/optimizer/sub_grad/Neg@optimizer/gradients_2/optimizer/sub_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
2optimizer/gradients_2/optimizer/sub_grad/Reshape_1Reshape.optimizer/gradients_2/optimizer/sub_grad/Sum_10optimizer/gradients_2/optimizer/sub_grad/Shape_1*
T0*
Tshape0
�
9optimizer/gradients_2/optimizer/sub_grad/tuple/group_depsNoOp1^optimizer/gradients_2/optimizer/sub_grad/Reshape3^optimizer/gradients_2/optimizer/sub_grad/Reshape_1
�
Aoptimizer/gradients_2/optimizer/sub_grad/tuple/control_dependencyIdentity0optimizer/gradients_2/optimizer/sub_grad/Reshape:^optimizer/gradients_2/optimizer/sub_grad/tuple/group_deps*
T0*C
_class9
75loc:@optimizer/gradients_2/optimizer/sub_grad/Reshape
�
Coptimizer/gradients_2/optimizer/sub_grad/tuple/control_dependency_1Identity2optimizer/gradients_2/optimizer/sub_grad/Reshape_1:^optimizer/gradients_2/optimizer/sub_grad/tuple/group_deps*
T0*E
_class;
97loc:@optimizer/gradients_2/optimizer/sub_grad/Reshape_1
c
0optimizer/gradients_2/optimizer/sub_1_grad/ShapeShapeoptimizer/Sum_7*
T0*
out_type0
x
2optimizer/gradients_2/optimizer/sub_1_grad/Shape_1Shape"optimizer/curiosity_value_estimate*
T0*
out_type0
�
@optimizer/gradients_2/optimizer/sub_1_grad/BroadcastGradientArgsBroadcastGradientArgs0optimizer/gradients_2/optimizer/sub_1_grad/Shape2optimizer/gradients_2/optimizer/sub_1_grad/Shape_1*
T0
�
.optimizer/gradients_2/optimizer/sub_1_grad/SumSumUoptimizer/gradients_2/optimizer/clip_by_value_1/Minimum_grad/tuple/control_dependency@optimizer/gradients_2/optimizer/sub_1_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
2optimizer/gradients_2/optimizer/sub_1_grad/ReshapeReshape.optimizer/gradients_2/optimizer/sub_1_grad/Sum0optimizer/gradients_2/optimizer/sub_1_grad/Shape*
T0*
Tshape0
�
.optimizer/gradients_2/optimizer/sub_1_grad/NegNegUoptimizer/gradients_2/optimizer/clip_by_value_1/Minimum_grad/tuple/control_dependency*
T0
�
0optimizer/gradients_2/optimizer/sub_1_grad/Sum_1Sum.optimizer/gradients_2/optimizer/sub_1_grad/NegBoptimizer/gradients_2/optimizer/sub_1_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
4optimizer/gradients_2/optimizer/sub_1_grad/Reshape_1Reshape0optimizer/gradients_2/optimizer/sub_1_grad/Sum_12optimizer/gradients_2/optimizer/sub_1_grad/Shape_1*
T0*
Tshape0
�
;optimizer/gradients_2/optimizer/sub_1_grad/tuple/group_depsNoOp3^optimizer/gradients_2/optimizer/sub_1_grad/Reshape5^optimizer/gradients_2/optimizer/sub_1_grad/Reshape_1
�
Coptimizer/gradients_2/optimizer/sub_1_grad/tuple/control_dependencyIdentity2optimizer/gradients_2/optimizer/sub_1_grad/Reshape<^optimizer/gradients_2/optimizer/sub_1_grad/tuple/group_deps*
T0*E
_class;
97loc:@optimizer/gradients_2/optimizer/sub_1_grad/Reshape
�
Eoptimizer/gradients_2/optimizer/sub_1_grad/tuple/control_dependency_1Identity4optimizer/gradients_2/optimizer/sub_1_grad/Reshape_1<^optimizer/gradients_2/optimizer/sub_1_grad/tuple/group_deps*
T0*G
_class=
;9loc:@optimizer/gradients_2/optimizer/sub_1_grad/Reshape_1
�
+optimizer/gradients_2/policy_1/Neg_grad/NegNegDoptimizer/gradients_2/policy_1/stack_1_grad/tuple/control_dependency*
T0
�
-optimizer/gradients_2/policy_1/Neg_1_grad/NegNegFoptimizer/gradients_2/policy_1/stack_1_grad/tuple/control_dependency_1*
T0
u
0optimizer/gradients_2/optimizer/Sum_5_grad/ShapeShape!optimizer/extrinsic_value/BiasAdd*
T0*
out_type0
�
/optimizer/gradients_2/optimizer/Sum_5_grad/SizeConst*C
_class9
75loc:@optimizer/gradients_2/optimizer/Sum_5_grad/Shape*
dtype0*
value	B :
�
.optimizer/gradients_2/optimizer/Sum_5_grad/addAddV2!optimizer/Sum_5/reduction_indices/optimizer/gradients_2/optimizer/Sum_5_grad/Size*
T0*C
_class9
75loc:@optimizer/gradients_2/optimizer/Sum_5_grad/Shape
�
.optimizer/gradients_2/optimizer/Sum_5_grad/modFloorMod.optimizer/gradients_2/optimizer/Sum_5_grad/add/optimizer/gradients_2/optimizer/Sum_5_grad/Size*
T0*C
_class9
75loc:@optimizer/gradients_2/optimizer/Sum_5_grad/Shape
�
2optimizer/gradients_2/optimizer/Sum_5_grad/Shape_1Const*C
_class9
75loc:@optimizer/gradients_2/optimizer/Sum_5_grad/Shape*
dtype0*
valueB 
�
6optimizer/gradients_2/optimizer/Sum_5_grad/range/startConst*C
_class9
75loc:@optimizer/gradients_2/optimizer/Sum_5_grad/Shape*
dtype0*
value	B : 
�
6optimizer/gradients_2/optimizer/Sum_5_grad/range/deltaConst*C
_class9
75loc:@optimizer/gradients_2/optimizer/Sum_5_grad/Shape*
dtype0*
value	B :
�
0optimizer/gradients_2/optimizer/Sum_5_grad/rangeRange6optimizer/gradients_2/optimizer/Sum_5_grad/range/start/optimizer/gradients_2/optimizer/Sum_5_grad/Size6optimizer/gradients_2/optimizer/Sum_5_grad/range/delta*

Tidx0*C
_class9
75loc:@optimizer/gradients_2/optimizer/Sum_5_grad/Shape
�
5optimizer/gradients_2/optimizer/Sum_5_grad/Fill/valueConst*C
_class9
75loc:@optimizer/gradients_2/optimizer/Sum_5_grad/Shape*
dtype0*
value	B :
�
/optimizer/gradients_2/optimizer/Sum_5_grad/FillFill2optimizer/gradients_2/optimizer/Sum_5_grad/Shape_15optimizer/gradients_2/optimizer/Sum_5_grad/Fill/value*
T0*C
_class9
75loc:@optimizer/gradients_2/optimizer/Sum_5_grad/Shape*

index_type0
�
8optimizer/gradients_2/optimizer/Sum_5_grad/DynamicStitchDynamicStitch0optimizer/gradients_2/optimizer/Sum_5_grad/range.optimizer/gradients_2/optimizer/Sum_5_grad/mod0optimizer/gradients_2/optimizer/Sum_5_grad/Shape/optimizer/gradients_2/optimizer/Sum_5_grad/Fill*
N*
T0*C
_class9
75loc:@optimizer/gradients_2/optimizer/Sum_5_grad/Shape
�
2optimizer/gradients_2/optimizer/Sum_5_grad/ReshapeReshapeAoptimizer/gradients_2/optimizer/sub_grad/tuple/control_dependency8optimizer/gradients_2/optimizer/Sum_5_grad/DynamicStitch*
T0*
Tshape0
�
6optimizer/gradients_2/optimizer/Sum_5_grad/BroadcastToBroadcastTo2optimizer/gradients_2/optimizer/Sum_5_grad/Reshape0optimizer/gradients_2/optimizer/Sum_5_grad/Shape*
T0*

Tidx0
u
0optimizer/gradients_2/optimizer/Sum_7_grad/ShapeShape!optimizer/curiosity_value/BiasAdd*
T0*
out_type0
�
/optimizer/gradients_2/optimizer/Sum_7_grad/SizeConst*C
_class9
75loc:@optimizer/gradients_2/optimizer/Sum_7_grad/Shape*
dtype0*
value	B :
�
.optimizer/gradients_2/optimizer/Sum_7_grad/addAddV2!optimizer/Sum_7/reduction_indices/optimizer/gradients_2/optimizer/Sum_7_grad/Size*
T0*C
_class9
75loc:@optimizer/gradients_2/optimizer/Sum_7_grad/Shape
�
.optimizer/gradients_2/optimizer/Sum_7_grad/modFloorMod.optimizer/gradients_2/optimizer/Sum_7_grad/add/optimizer/gradients_2/optimizer/Sum_7_grad/Size*
T0*C
_class9
75loc:@optimizer/gradients_2/optimizer/Sum_7_grad/Shape
�
2optimizer/gradients_2/optimizer/Sum_7_grad/Shape_1Const*C
_class9
75loc:@optimizer/gradients_2/optimizer/Sum_7_grad/Shape*
dtype0*
valueB 
�
6optimizer/gradients_2/optimizer/Sum_7_grad/range/startConst*C
_class9
75loc:@optimizer/gradients_2/optimizer/Sum_7_grad/Shape*
dtype0*
value	B : 
�
6optimizer/gradients_2/optimizer/Sum_7_grad/range/deltaConst*C
_class9
75loc:@optimizer/gradients_2/optimizer/Sum_7_grad/Shape*
dtype0*
value	B :
�
0optimizer/gradients_2/optimizer/Sum_7_grad/rangeRange6optimizer/gradients_2/optimizer/Sum_7_grad/range/start/optimizer/gradients_2/optimizer/Sum_7_grad/Size6optimizer/gradients_2/optimizer/Sum_7_grad/range/delta*

Tidx0*C
_class9
75loc:@optimizer/gradients_2/optimizer/Sum_7_grad/Shape
�
5optimizer/gradients_2/optimizer/Sum_7_grad/Fill/valueConst*C
_class9
75loc:@optimizer/gradients_2/optimizer/Sum_7_grad/Shape*
dtype0*
value	B :
�
/optimizer/gradients_2/optimizer/Sum_7_grad/FillFill2optimizer/gradients_2/optimizer/Sum_7_grad/Shape_15optimizer/gradients_2/optimizer/Sum_7_grad/Fill/value*
T0*C
_class9
75loc:@optimizer/gradients_2/optimizer/Sum_7_grad/Shape*

index_type0
�
8optimizer/gradients_2/optimizer/Sum_7_grad/DynamicStitchDynamicStitch0optimizer/gradients_2/optimizer/Sum_7_grad/range.optimizer/gradients_2/optimizer/Sum_7_grad/mod0optimizer/gradients_2/optimizer/Sum_7_grad/Shape/optimizer/gradients_2/optimizer/Sum_7_grad/Fill*
N*
T0*C
_class9
75loc:@optimizer/gradients_2/optimizer/Sum_7_grad/Shape
�
2optimizer/gradients_2/optimizer/Sum_7_grad/ReshapeReshapeCoptimizer/gradients_2/optimizer/sub_1_grad/tuple/control_dependency8optimizer/gradients_2/optimizer/Sum_7_grad/DynamicStitch*
T0*
Tshape0
�
6optimizer/gradients_2/optimizer/Sum_7_grad/BroadcastToBroadcastTo2optimizer/gradients_2/optimizer/Sum_7_grad/Reshape0optimizer/gradients_2/optimizer/Sum_7_grad/Shape*
T0*

Tidx0
�
Woptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_2/Reshape_2_grad/ShapeShape,policy_1/softmax_cross_entropy_with_logits_2*
T0*
out_type0
�
Yoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_2/Reshape_2_grad/ReshapeReshape+optimizer/gradients_2/policy_1/Neg_grad/NegWoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_2/Reshape_2_grad/Shape*
T0*
Tshape0
�
Woptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_3/Reshape_2_grad/ShapeShape,policy_1/softmax_cross_entropy_with_logits_3*
T0*
out_type0
�
Yoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_3/Reshape_2_grad/ReshapeReshape-optimizer/gradients_2/policy_1/Neg_1_grad/NegWoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_3/Reshape_2_grad/Shape*
T0*
Tshape0
�
optimizer/gradients_2/AddN_1AddN6optimizer/gradients_2/optimizer/Sum_6_grad/BroadcastTo6optimizer/gradients_2/optimizer/Sum_5_grad/BroadcastTo*
N*
T0*I
_class?
=;loc:@optimizer/gradients_2/optimizer/Sum_6_grad/BroadcastTo
�
Hoptimizer/gradients_2/optimizer/extrinsic_value/BiasAdd_grad/BiasAddGradBiasAddGradoptimizer/gradients_2/AddN_1*
T0*
data_formatNHWC
�
Moptimizer/gradients_2/optimizer/extrinsic_value/BiasAdd_grad/tuple/group_depsNoOp^optimizer/gradients_2/AddN_1I^optimizer/gradients_2/optimizer/extrinsic_value/BiasAdd_grad/BiasAddGrad
�
Uoptimizer/gradients_2/optimizer/extrinsic_value/BiasAdd_grad/tuple/control_dependencyIdentityoptimizer/gradients_2/AddN_1N^optimizer/gradients_2/optimizer/extrinsic_value/BiasAdd_grad/tuple/group_deps*
T0*I
_class?
=;loc:@optimizer/gradients_2/optimizer/Sum_6_grad/BroadcastTo
�
Woptimizer/gradients_2/optimizer/extrinsic_value/BiasAdd_grad/tuple/control_dependency_1IdentityHoptimizer/gradients_2/optimizer/extrinsic_value/BiasAdd_grad/BiasAddGradN^optimizer/gradients_2/optimizer/extrinsic_value/BiasAdd_grad/tuple/group_deps*
T0*[
_classQ
OMloc:@optimizer/gradients_2/optimizer/extrinsic_value/BiasAdd_grad/BiasAddGrad
�
optimizer/gradients_2/AddN_2AddN6optimizer/gradients_2/optimizer/Sum_8_grad/BroadcastTo6optimizer/gradients_2/optimizer/Sum_7_grad/BroadcastTo*
N*
T0*I
_class?
=;loc:@optimizer/gradients_2/optimizer/Sum_8_grad/BroadcastTo
�
Hoptimizer/gradients_2/optimizer/curiosity_value/BiasAdd_grad/BiasAddGradBiasAddGradoptimizer/gradients_2/AddN_2*
T0*
data_formatNHWC
�
Moptimizer/gradients_2/optimizer/curiosity_value/BiasAdd_grad/tuple/group_depsNoOp^optimizer/gradients_2/AddN_2I^optimizer/gradients_2/optimizer/curiosity_value/BiasAdd_grad/BiasAddGrad
�
Uoptimizer/gradients_2/optimizer/curiosity_value/BiasAdd_grad/tuple/control_dependencyIdentityoptimizer/gradients_2/AddN_2N^optimizer/gradients_2/optimizer/curiosity_value/BiasAdd_grad/tuple/group_deps*
T0*I
_class?
=;loc:@optimizer/gradients_2/optimizer/Sum_8_grad/BroadcastTo
�
Woptimizer/gradients_2/optimizer/curiosity_value/BiasAdd_grad/tuple/control_dependency_1IdentityHoptimizer/gradients_2/optimizer/curiosity_value/BiasAdd_grad/BiasAddGradN^optimizer/gradients_2/optimizer/curiosity_value/BiasAdd_grad/tuple/group_deps*
T0*[
_classQ
OMloc:@optimizer/gradients_2/optimizer/curiosity_value/BiasAdd_grad/BiasAddGrad
h
"optimizer/gradients_2/zeros_like_6	ZerosLike.policy_1/softmax_cross_entropy_with_logits_2:1*
T0
�
Voptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_2_grad/ExpandDims/dimConst*
dtype0*
valueB :
���������
�
Roptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_2_grad/ExpandDims
ExpandDimsYoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_2/Reshape_2_grad/ReshapeVoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_2_grad/ExpandDims/dim*
T0*

Tdim0
�
Koptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_2_grad/mulMulRoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_2_grad/ExpandDims.policy_1/softmax_cross_entropy_with_logits_2:1*
T0
�
Roptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_2_grad/LogSoftmax
LogSoftmax4policy_1/softmax_cross_entropy_with_logits_2/Reshape*
T0
�
Koptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_2_grad/NegNegRoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_2_grad/LogSoftmax*
T0
�
Xoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_2_grad/ExpandDims_1/dimConst*
dtype0*
valueB :
���������
�
Toptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_2_grad/ExpandDims_1
ExpandDimsYoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_2/Reshape_2_grad/ReshapeXoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_2_grad/ExpandDims_1/dim*
T0*

Tdim0
�
Moptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_2_grad/mul_1MulToptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_2_grad/ExpandDims_1Koptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_2_grad/Neg*
T0
�
Xoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_2_grad/tuple/group_depsNoOpL^optimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_2_grad/mulN^optimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_2_grad/mul_1
�
`optimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_2_grad/tuple/control_dependencyIdentityKoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_2_grad/mulY^optimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_2_grad/tuple/group_deps*
T0*^
_classT
RPloc:@optimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_2_grad/mul
�
boptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_2_grad/tuple/control_dependency_1IdentityMoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_2_grad/mul_1Y^optimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_2_grad/tuple/group_deps*
T0*`
_classV
TRloc:@optimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_2_grad/mul_1
h
"optimizer/gradients_2/zeros_like_7	ZerosLike.policy_1/softmax_cross_entropy_with_logits_3:1*
T0
�
Voptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_3_grad/ExpandDims/dimConst*
dtype0*
valueB :
���������
�
Roptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_3_grad/ExpandDims
ExpandDimsYoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_3/Reshape_2_grad/ReshapeVoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_3_grad/ExpandDims/dim*
T0*

Tdim0
�
Koptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_3_grad/mulMulRoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_3_grad/ExpandDims.policy_1/softmax_cross_entropy_with_logits_3:1*
T0
�
Roptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_3_grad/LogSoftmax
LogSoftmax4policy_1/softmax_cross_entropy_with_logits_3/Reshape*
T0
�
Koptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_3_grad/NegNegRoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_3_grad/LogSoftmax*
T0
�
Xoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_3_grad/ExpandDims_1/dimConst*
dtype0*
valueB :
���������
�
Toptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_3_grad/ExpandDims_1
ExpandDimsYoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_3/Reshape_2_grad/ReshapeXoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_3_grad/ExpandDims_1/dim*
T0*

Tdim0
�
Moptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_3_grad/mul_1MulToptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_3_grad/ExpandDims_1Koptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_3_grad/Neg*
T0
�
Xoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_3_grad/tuple/group_depsNoOpL^optimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_3_grad/mulN^optimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_3_grad/mul_1
�
`optimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_3_grad/tuple/control_dependencyIdentityKoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_3_grad/mulY^optimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_3_grad/tuple/group_deps*
T0*^
_classT
RPloc:@optimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_3_grad/mul
�
boptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_3_grad/tuple/control_dependency_1IdentityMoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_3_grad/mul_1Y^optimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_3_grad/tuple/group_deps*
T0*`
_classV
TRloc:@optimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_3_grad/mul_1
�
Boptimizer/gradients_2/optimizer/extrinsic_value/MatMul_grad/MatMulMatMulUoptimizer/gradients_2/optimizer/extrinsic_value/BiasAdd_grad/tuple/control_dependency&optimizer//extrinsic_value/kernel/read*
T0*
transpose_a( *
transpose_b(
�
Doptimizer/gradients_2/optimizer/extrinsic_value/MatMul_grad/MatMul_1MatMul#optimizer/main_graph_0/hidden_1/MulUoptimizer/gradients_2/optimizer/extrinsic_value/BiasAdd_grad/tuple/control_dependency*
T0*
transpose_a(*
transpose_b( 
�
Loptimizer/gradients_2/optimizer/extrinsic_value/MatMul_grad/tuple/group_depsNoOpC^optimizer/gradients_2/optimizer/extrinsic_value/MatMul_grad/MatMulE^optimizer/gradients_2/optimizer/extrinsic_value/MatMul_grad/MatMul_1
�
Toptimizer/gradients_2/optimizer/extrinsic_value/MatMul_grad/tuple/control_dependencyIdentityBoptimizer/gradients_2/optimizer/extrinsic_value/MatMul_grad/MatMulM^optimizer/gradients_2/optimizer/extrinsic_value/MatMul_grad/tuple/group_deps*
T0*U
_classK
IGloc:@optimizer/gradients_2/optimizer/extrinsic_value/MatMul_grad/MatMul
�
Voptimizer/gradients_2/optimizer/extrinsic_value/MatMul_grad/tuple/control_dependency_1IdentityDoptimizer/gradients_2/optimizer/extrinsic_value/MatMul_grad/MatMul_1M^optimizer/gradients_2/optimizer/extrinsic_value/MatMul_grad/tuple/group_deps*
T0*W
_classM
KIloc:@optimizer/gradients_2/optimizer/extrinsic_value/MatMul_grad/MatMul_1
�
Boptimizer/gradients_2/optimizer/curiosity_value/MatMul_grad/MatMulMatMulUoptimizer/gradients_2/optimizer/curiosity_value/BiasAdd_grad/tuple/control_dependency&optimizer//curiosity_value/kernel/read*
T0*
transpose_a( *
transpose_b(
�
Doptimizer/gradients_2/optimizer/curiosity_value/MatMul_grad/MatMul_1MatMul#optimizer/main_graph_0/hidden_1/MulUoptimizer/gradients_2/optimizer/curiosity_value/BiasAdd_grad/tuple/control_dependency*
T0*
transpose_a(*
transpose_b( 
�
Loptimizer/gradients_2/optimizer/curiosity_value/MatMul_grad/tuple/group_depsNoOpC^optimizer/gradients_2/optimizer/curiosity_value/MatMul_grad/MatMulE^optimizer/gradients_2/optimizer/curiosity_value/MatMul_grad/MatMul_1
�
Toptimizer/gradients_2/optimizer/curiosity_value/MatMul_grad/tuple/control_dependencyIdentityBoptimizer/gradients_2/optimizer/curiosity_value/MatMul_grad/MatMulM^optimizer/gradients_2/optimizer/curiosity_value/MatMul_grad/tuple/group_deps*
T0*U
_classK
IGloc:@optimizer/gradients_2/optimizer/curiosity_value/MatMul_grad/MatMul
�
Voptimizer/gradients_2/optimizer/curiosity_value/MatMul_grad/tuple/control_dependency_1IdentityDoptimizer/gradients_2/optimizer/curiosity_value/MatMul_grad/MatMul_1M^optimizer/gradients_2/optimizer/curiosity_value/MatMul_grad/tuple/group_deps*
T0*W
_classM
KIloc:@optimizer/gradients_2/optimizer/curiosity_value/MatMul_grad/MatMul_1
�
Uoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_2/Reshape_grad/ShapeShapepolicy_1/strided_slice_9*
T0*
out_type0
�
Woptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_2/Reshape_grad/ReshapeReshape`optimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_2_grad/tuple/control_dependencyUoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_2/Reshape_grad/Shape*
T0*
Tshape0
�
Uoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_3/Reshape_grad/ShapeShapepolicy_1/strided_slice_11*
T0*
out_type0
�
Woptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_3/Reshape_grad/ReshapeReshape`optimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_3_grad/tuple/control_dependencyUoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_3/Reshape_grad/Shape*
T0*
Tshape0
�
optimizer/gradients_2/AddN_3AddNToptimizer/gradients_2/optimizer/extrinsic_value/MatMul_grad/tuple/control_dependencyToptimizer/gradients_2/optimizer/curiosity_value/MatMul_grad/tuple/control_dependency*
N*
T0*U
_classK
IGloc:@optimizer/gradients_2/optimizer/extrinsic_value/MatMul_grad/MatMul
�
Doptimizer/gradients_2/optimizer/main_graph_0/hidden_1/Mul_grad/ShapeShape'optimizer/main_graph_0/hidden_1/BiasAdd*
T0*
out_type0
�
Foptimizer/gradients_2/optimizer/main_graph_0/hidden_1/Mul_grad/Shape_1Shape'optimizer/main_graph_0/hidden_1/Sigmoid*
T0*
out_type0
�
Toptimizer/gradients_2/optimizer/main_graph_0/hidden_1/Mul_grad/BroadcastGradientArgsBroadcastGradientArgsDoptimizer/gradients_2/optimizer/main_graph_0/hidden_1/Mul_grad/ShapeFoptimizer/gradients_2/optimizer/main_graph_0/hidden_1/Mul_grad/Shape_1*
T0
�
Boptimizer/gradients_2/optimizer/main_graph_0/hidden_1/Mul_grad/MulMuloptimizer/gradients_2/AddN_3'optimizer/main_graph_0/hidden_1/Sigmoid*
T0
�
Boptimizer/gradients_2/optimizer/main_graph_0/hidden_1/Mul_grad/SumSumBoptimizer/gradients_2/optimizer/main_graph_0/hidden_1/Mul_grad/MulToptimizer/gradients_2/optimizer/main_graph_0/hidden_1/Mul_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
Foptimizer/gradients_2/optimizer/main_graph_0/hidden_1/Mul_grad/ReshapeReshapeBoptimizer/gradients_2/optimizer/main_graph_0/hidden_1/Mul_grad/SumDoptimizer/gradients_2/optimizer/main_graph_0/hidden_1/Mul_grad/Shape*
T0*
Tshape0
�
Doptimizer/gradients_2/optimizer/main_graph_0/hidden_1/Mul_grad/Mul_1Mul'optimizer/main_graph_0/hidden_1/BiasAddoptimizer/gradients_2/AddN_3*
T0
�
Doptimizer/gradients_2/optimizer/main_graph_0/hidden_1/Mul_grad/Sum_1SumDoptimizer/gradients_2/optimizer/main_graph_0/hidden_1/Mul_grad/Mul_1Voptimizer/gradients_2/optimizer/main_graph_0/hidden_1/Mul_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
Hoptimizer/gradients_2/optimizer/main_graph_0/hidden_1/Mul_grad/Reshape_1ReshapeDoptimizer/gradients_2/optimizer/main_graph_0/hidden_1/Mul_grad/Sum_1Foptimizer/gradients_2/optimizer/main_graph_0/hidden_1/Mul_grad/Shape_1*
T0*
Tshape0
�
Ooptimizer/gradients_2/optimizer/main_graph_0/hidden_1/Mul_grad/tuple/group_depsNoOpG^optimizer/gradients_2/optimizer/main_graph_0/hidden_1/Mul_grad/ReshapeI^optimizer/gradients_2/optimizer/main_graph_0/hidden_1/Mul_grad/Reshape_1
�
Woptimizer/gradients_2/optimizer/main_graph_0/hidden_1/Mul_grad/tuple/control_dependencyIdentityFoptimizer/gradients_2/optimizer/main_graph_0/hidden_1/Mul_grad/ReshapeP^optimizer/gradients_2/optimizer/main_graph_0/hidden_1/Mul_grad/tuple/group_deps*
T0*Y
_classO
MKloc:@optimizer/gradients_2/optimizer/main_graph_0/hidden_1/Mul_grad/Reshape
�
Yoptimizer/gradients_2/optimizer/main_graph_0/hidden_1/Mul_grad/tuple/control_dependency_1IdentityHoptimizer/gradients_2/optimizer/main_graph_0/hidden_1/Mul_grad/Reshape_1P^optimizer/gradients_2/optimizer/main_graph_0/hidden_1/Mul_grad/tuple/group_deps*
T0*[
_classQ
OMloc:@optimizer/gradients_2/optimizer/main_graph_0/hidden_1/Mul_grad/Reshape_1
n
9optimizer/gradients_2/policy_1/strided_slice_9_grad/ShapeShapepolicy_1/concat_2*
T0*
out_type0

Joptimizer/gradients_2/policy_1/strided_slice_9_grad/StridedSliceGrad/beginConst*
dtype0*
valueB"        
}
Hoptimizer/gradients_2/policy_1/strided_slice_9_grad/StridedSliceGrad/endConst*
dtype0*
valueB"       
�
Loptimizer/gradients_2/policy_1/strided_slice_9_grad/StridedSliceGrad/stridesConst*
dtype0*
valueB"      
�
Doptimizer/gradients_2/policy_1/strided_slice_9_grad/StridedSliceGradStridedSliceGrad9optimizer/gradients_2/policy_1/strided_slice_9_grad/ShapeJoptimizer/gradients_2/policy_1/strided_slice_9_grad/StridedSliceGrad/beginHoptimizer/gradients_2/policy_1/strided_slice_9_grad/StridedSliceGrad/endLoptimizer/gradients_2/policy_1/strided_slice_9_grad/StridedSliceGrad/stridesWoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_2/Reshape_grad/Reshape*
Index0*
T0*

begin_mask*
ellipsis_mask *
end_mask*
new_axis_mask *
shrink_axis_mask 
o
:optimizer/gradients_2/policy_1/strided_slice_11_grad/ShapeShapepolicy_1/concat_2*
T0*
out_type0
�
Koptimizer/gradients_2/policy_1/strided_slice_11_grad/StridedSliceGrad/beginConst*
dtype0*
valueB"       
~
Ioptimizer/gradients_2/policy_1/strided_slice_11_grad/StridedSliceGrad/endConst*
dtype0*
valueB"       
�
Moptimizer/gradients_2/policy_1/strided_slice_11_grad/StridedSliceGrad/stridesConst*
dtype0*
valueB"      
�
Eoptimizer/gradients_2/policy_1/strided_slice_11_grad/StridedSliceGradStridedSliceGrad:optimizer/gradients_2/policy_1/strided_slice_11_grad/ShapeKoptimizer/gradients_2/policy_1/strided_slice_11_grad/StridedSliceGrad/beginIoptimizer/gradients_2/policy_1/strided_slice_11_grad/StridedSliceGrad/endMoptimizer/gradients_2/policy_1/strided_slice_11_grad/StridedSliceGrad/stridesWoptimizer/gradients_2/policy_1/softmax_cross_entropy_with_logits_3/Reshape_grad/Reshape*
Index0*
T0*

begin_mask*
ellipsis_mask *
end_mask*
new_axis_mask *
shrink_axis_mask 
�
Noptimizer/gradients_2/optimizer/main_graph_0/hidden_1/Sigmoid_grad/SigmoidGradSigmoidGrad'optimizer/main_graph_0/hidden_1/SigmoidYoptimizer/gradients_2/optimizer/main_graph_0/hidden_1/Mul_grad/tuple/control_dependency_1*
T0
�
optimizer/gradients_2/AddN_4AddNDoptimizer/gradients_2/policy_1/strided_slice_5_grad/StridedSliceGradDoptimizer/gradients_2/policy_1/strided_slice_7_grad/StridedSliceGradDoptimizer/gradients_2/policy_1/strided_slice_4_grad/StridedSliceGradDoptimizer/gradients_2/policy_1/strided_slice_6_grad/StridedSliceGradDoptimizer/gradients_2/policy_1/strided_slice_9_grad/StridedSliceGradEoptimizer/gradients_2/policy_1/strided_slice_11_grad/StridedSliceGrad*
N*
T0*W
_classM
KIloc:@optimizer/gradients_2/policy_1/strided_slice_5_grad/StridedSliceGrad
[
1optimizer/gradients_2/policy_1/concat_2_grad/RankConst*
dtype0*
value	B :
�
0optimizer/gradients_2/policy_1/concat_2_grad/modFloorModpolicy_1/concat_2/axis1optimizer/gradients_2/policy_1/concat_2_grad/Rank*
T0
d
2optimizer/gradients_2/policy_1/concat_2_grad/ShapeShapepolicy_1/Log_2*
T0*
out_type0

3optimizer/gradients_2/policy_1/concat_2_grad/ShapeNShapeNpolicy_1/Log_2policy_1/Log_3*
N*
T0*
out_type0
�
9optimizer/gradients_2/policy_1/concat_2_grad/ConcatOffsetConcatOffset0optimizer/gradients_2/policy_1/concat_2_grad/mod3optimizer/gradients_2/policy_1/concat_2_grad/ShapeN5optimizer/gradients_2/policy_1/concat_2_grad/ShapeN:1*
N
�
2optimizer/gradients_2/policy_1/concat_2_grad/SliceSliceoptimizer/gradients_2/AddN_49optimizer/gradients_2/policy_1/concat_2_grad/ConcatOffset3optimizer/gradients_2/policy_1/concat_2_grad/ShapeN*
Index0*
T0
�
4optimizer/gradients_2/policy_1/concat_2_grad/Slice_1Sliceoptimizer/gradients_2/AddN_4;optimizer/gradients_2/policy_1/concat_2_grad/ConcatOffset:15optimizer/gradients_2/policy_1/concat_2_grad/ShapeN:1*
Index0*
T0
�
=optimizer/gradients_2/policy_1/concat_2_grad/tuple/group_depsNoOp3^optimizer/gradients_2/policy_1/concat_2_grad/Slice5^optimizer/gradients_2/policy_1/concat_2_grad/Slice_1
�
Eoptimizer/gradients_2/policy_1/concat_2_grad/tuple/control_dependencyIdentity2optimizer/gradients_2/policy_1/concat_2_grad/Slice>^optimizer/gradients_2/policy_1/concat_2_grad/tuple/group_deps*
T0*E
_class;
97loc:@optimizer/gradients_2/policy_1/concat_2_grad/Slice
�
Goptimizer/gradients_2/policy_1/concat_2_grad/tuple/control_dependency_1Identity4optimizer/gradients_2/policy_1/concat_2_grad/Slice_1>^optimizer/gradients_2/policy_1/concat_2_grad/tuple/group_deps*
T0*G
_class=
;9loc:@optimizer/gradients_2/policy_1/concat_2_grad/Slice_1
�
optimizer/gradients_2/AddN_5AddNWoptimizer/gradients_2/optimizer/main_graph_0/hidden_1/Mul_grad/tuple/control_dependencyNoptimizer/gradients_2/optimizer/main_graph_0/hidden_1/Sigmoid_grad/SigmoidGrad*
N*
T0*Y
_classO
MKloc:@optimizer/gradients_2/optimizer/main_graph_0/hidden_1/Mul_grad/Reshape
�
Noptimizer/gradients_2/optimizer/main_graph_0/hidden_1/BiasAdd_grad/BiasAddGradBiasAddGradoptimizer/gradients_2/AddN_5*
T0*
data_formatNHWC
�
Soptimizer/gradients_2/optimizer/main_graph_0/hidden_1/BiasAdd_grad/tuple/group_depsNoOp^optimizer/gradients_2/AddN_5O^optimizer/gradients_2/optimizer/main_graph_0/hidden_1/BiasAdd_grad/BiasAddGrad
�
[optimizer/gradients_2/optimizer/main_graph_0/hidden_1/BiasAdd_grad/tuple/control_dependencyIdentityoptimizer/gradients_2/AddN_5T^optimizer/gradients_2/optimizer/main_graph_0/hidden_1/BiasAdd_grad/tuple/group_deps*
T0*Y
_classO
MKloc:@optimizer/gradients_2/optimizer/main_graph_0/hidden_1/Mul_grad/Reshape
�
]optimizer/gradients_2/optimizer/main_graph_0/hidden_1/BiasAdd_grad/tuple/control_dependency_1IdentityNoptimizer/gradients_2/optimizer/main_graph_0/hidden_1/BiasAdd_grad/BiasAddGradT^optimizer/gradients_2/optimizer/main_graph_0/hidden_1/BiasAdd_grad/tuple/group_deps*
T0*a
_classW
USloc:@optimizer/gradients_2/optimizer/main_graph_0/hidden_1/BiasAdd_grad/BiasAddGrad
�
4optimizer/gradients_2/policy_1/Log_2_grad/Reciprocal
Reciprocalpolicy_1/add_4F^optimizer/gradients_2/policy_1/concat_2_grad/tuple/control_dependency*
T0
�
-optimizer/gradients_2/policy_1/Log_2_grad/mulMulEoptimizer/gradients_2/policy_1/concat_2_grad/tuple/control_dependency4optimizer/gradients_2/policy_1/Log_2_grad/Reciprocal*
T0
�
4optimizer/gradients_2/policy_1/Log_3_grad/Reciprocal
Reciprocalpolicy_1/add_5H^optimizer/gradients_2/policy_1/concat_2_grad/tuple/control_dependency_1*
T0
�
-optimizer/gradients_2/policy_1/Log_3_grad/mulMulGoptimizer/gradients_2/policy_1/concat_2_grad/tuple/control_dependency_14optimizer/gradients_2/policy_1/Log_3_grad/Reciprocal*
T0
�
Hoptimizer/gradients_2/optimizer/main_graph_0/hidden_1/MatMul_grad/MatMulMatMul[optimizer/gradients_2/optimizer/main_graph_0/hidden_1/BiasAdd_grad/tuple/control_dependency,optimizer//main_graph_0/hidden_1/kernel/read*
T0*
transpose_a( *
transpose_b(
�
Joptimizer/gradients_2/optimizer/main_graph_0/hidden_1/MatMul_grad/MatMul_1MatMul#optimizer/main_graph_0/hidden_0/Mul[optimizer/gradients_2/optimizer/main_graph_0/hidden_1/BiasAdd_grad/tuple/control_dependency*
T0*
transpose_a(*
transpose_b( 
�
Roptimizer/gradients_2/optimizer/main_graph_0/hidden_1/MatMul_grad/tuple/group_depsNoOpI^optimizer/gradients_2/optimizer/main_graph_0/hidden_1/MatMul_grad/MatMulK^optimizer/gradients_2/optimizer/main_graph_0/hidden_1/MatMul_grad/MatMul_1
�
Zoptimizer/gradients_2/optimizer/main_graph_0/hidden_1/MatMul_grad/tuple/control_dependencyIdentityHoptimizer/gradients_2/optimizer/main_graph_0/hidden_1/MatMul_grad/MatMulS^optimizer/gradients_2/optimizer/main_graph_0/hidden_1/MatMul_grad/tuple/group_deps*
T0*[
_classQ
OMloc:@optimizer/gradients_2/optimizer/main_graph_0/hidden_1/MatMul_grad/MatMul
�
\optimizer/gradients_2/optimizer/main_graph_0/hidden_1/MatMul_grad/tuple/control_dependency_1IdentityJoptimizer/gradients_2/optimizer/main_graph_0/hidden_1/MatMul_grad/MatMul_1S^optimizer/gradients_2/optimizer/main_graph_0/hidden_1/MatMul_grad/tuple/group_deps*
T0*]
_classS
QOloc:@optimizer/gradients_2/optimizer/main_graph_0/hidden_1/MatMul_grad/MatMul_1
c
/optimizer/gradients_2/policy_1/add_4_grad/ShapeShapepolicy_1/truediv*
T0*
out_type0
e
1optimizer/gradients_2/policy_1/add_4_grad/Shape_1Shapepolicy_1/add_4/y*
T0*
out_type0
�
?optimizer/gradients_2/policy_1/add_4_grad/BroadcastGradientArgsBroadcastGradientArgs/optimizer/gradients_2/policy_1/add_4_grad/Shape1optimizer/gradients_2/policy_1/add_4_grad/Shape_1*
T0
�
-optimizer/gradients_2/policy_1/add_4_grad/SumSum-optimizer/gradients_2/policy_1/Log_2_grad/mul?optimizer/gradients_2/policy_1/add_4_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
1optimizer/gradients_2/policy_1/add_4_grad/ReshapeReshape-optimizer/gradients_2/policy_1/add_4_grad/Sum/optimizer/gradients_2/policy_1/add_4_grad/Shape*
T0*
Tshape0
�
/optimizer/gradients_2/policy_1/add_4_grad/Sum_1Sum-optimizer/gradients_2/policy_1/Log_2_grad/mulAoptimizer/gradients_2/policy_1/add_4_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
3optimizer/gradients_2/policy_1/add_4_grad/Reshape_1Reshape/optimizer/gradients_2/policy_1/add_4_grad/Sum_11optimizer/gradients_2/policy_1/add_4_grad/Shape_1*
T0*
Tshape0
�
:optimizer/gradients_2/policy_1/add_4_grad/tuple/group_depsNoOp2^optimizer/gradients_2/policy_1/add_4_grad/Reshape4^optimizer/gradients_2/policy_1/add_4_grad/Reshape_1
�
Boptimizer/gradients_2/policy_1/add_4_grad/tuple/control_dependencyIdentity1optimizer/gradients_2/policy_1/add_4_grad/Reshape;^optimizer/gradients_2/policy_1/add_4_grad/tuple/group_deps*
T0*D
_class:
86loc:@optimizer/gradients_2/policy_1/add_4_grad/Reshape
�
Doptimizer/gradients_2/policy_1/add_4_grad/tuple/control_dependency_1Identity3optimizer/gradients_2/policy_1/add_4_grad/Reshape_1;^optimizer/gradients_2/policy_1/add_4_grad/tuple/group_deps*
T0*F
_class<
:8loc:@optimizer/gradients_2/policy_1/add_4_grad/Reshape_1
e
/optimizer/gradients_2/policy_1/add_5_grad/ShapeShapepolicy_1/truediv_1*
T0*
out_type0
e
1optimizer/gradients_2/policy_1/add_5_grad/Shape_1Shapepolicy_1/add_5/y*
T0*
out_type0
�
?optimizer/gradients_2/policy_1/add_5_grad/BroadcastGradientArgsBroadcastGradientArgs/optimizer/gradients_2/policy_1/add_5_grad/Shape1optimizer/gradients_2/policy_1/add_5_grad/Shape_1*
T0
�
-optimizer/gradients_2/policy_1/add_5_grad/SumSum-optimizer/gradients_2/policy_1/Log_3_grad/mul?optimizer/gradients_2/policy_1/add_5_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
1optimizer/gradients_2/policy_1/add_5_grad/ReshapeReshape-optimizer/gradients_2/policy_1/add_5_grad/Sum/optimizer/gradients_2/policy_1/add_5_grad/Shape*
T0*
Tshape0
�
/optimizer/gradients_2/policy_1/add_5_grad/Sum_1Sum-optimizer/gradients_2/policy_1/Log_3_grad/mulAoptimizer/gradients_2/policy_1/add_5_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
3optimizer/gradients_2/policy_1/add_5_grad/Reshape_1Reshape/optimizer/gradients_2/policy_1/add_5_grad/Sum_11optimizer/gradients_2/policy_1/add_5_grad/Shape_1*
T0*
Tshape0
�
:optimizer/gradients_2/policy_1/add_5_grad/tuple/group_depsNoOp2^optimizer/gradients_2/policy_1/add_5_grad/Reshape4^optimizer/gradients_2/policy_1/add_5_grad/Reshape_1
�
Boptimizer/gradients_2/policy_1/add_5_grad/tuple/control_dependencyIdentity1optimizer/gradients_2/policy_1/add_5_grad/Reshape;^optimizer/gradients_2/policy_1/add_5_grad/tuple/group_deps*
T0*D
_class:
86loc:@optimizer/gradients_2/policy_1/add_5_grad/Reshape
�
Doptimizer/gradients_2/policy_1/add_5_grad/tuple/control_dependency_1Identity3optimizer/gradients_2/policy_1/add_5_grad/Reshape_1;^optimizer/gradients_2/policy_1/add_5_grad/tuple/group_deps*
T0*F
_class<
:8loc:@optimizer/gradients_2/policy_1/add_5_grad/Reshape_1
�
Doptimizer/gradients_2/optimizer/main_graph_0/hidden_0/Mul_grad/ShapeShape'optimizer/main_graph_0/hidden_0/BiasAdd*
T0*
out_type0
�
Foptimizer/gradients_2/optimizer/main_graph_0/hidden_0/Mul_grad/Shape_1Shape'optimizer/main_graph_0/hidden_0/Sigmoid*
T0*
out_type0
�
Toptimizer/gradients_2/optimizer/main_graph_0/hidden_0/Mul_grad/BroadcastGradientArgsBroadcastGradientArgsDoptimizer/gradients_2/optimizer/main_graph_0/hidden_0/Mul_grad/ShapeFoptimizer/gradients_2/optimizer/main_graph_0/hidden_0/Mul_grad/Shape_1*
T0
�
Boptimizer/gradients_2/optimizer/main_graph_0/hidden_0/Mul_grad/MulMulZoptimizer/gradients_2/optimizer/main_graph_0/hidden_1/MatMul_grad/tuple/control_dependency'optimizer/main_graph_0/hidden_0/Sigmoid*
T0
�
Boptimizer/gradients_2/optimizer/main_graph_0/hidden_0/Mul_grad/SumSumBoptimizer/gradients_2/optimizer/main_graph_0/hidden_0/Mul_grad/MulToptimizer/gradients_2/optimizer/main_graph_0/hidden_0/Mul_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
Foptimizer/gradients_2/optimizer/main_graph_0/hidden_0/Mul_grad/ReshapeReshapeBoptimizer/gradients_2/optimizer/main_graph_0/hidden_0/Mul_grad/SumDoptimizer/gradients_2/optimizer/main_graph_0/hidden_0/Mul_grad/Shape*
T0*
Tshape0
�
Doptimizer/gradients_2/optimizer/main_graph_0/hidden_0/Mul_grad/Mul_1Mul'optimizer/main_graph_0/hidden_0/BiasAddZoptimizer/gradients_2/optimizer/main_graph_0/hidden_1/MatMul_grad/tuple/control_dependency*
T0
�
Doptimizer/gradients_2/optimizer/main_graph_0/hidden_0/Mul_grad/Sum_1SumDoptimizer/gradients_2/optimizer/main_graph_0/hidden_0/Mul_grad/Mul_1Voptimizer/gradients_2/optimizer/main_graph_0/hidden_0/Mul_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
Hoptimizer/gradients_2/optimizer/main_graph_0/hidden_0/Mul_grad/Reshape_1ReshapeDoptimizer/gradients_2/optimizer/main_graph_0/hidden_0/Mul_grad/Sum_1Foptimizer/gradients_2/optimizer/main_graph_0/hidden_0/Mul_grad/Shape_1*
T0*
Tshape0
�
Ooptimizer/gradients_2/optimizer/main_graph_0/hidden_0/Mul_grad/tuple/group_depsNoOpG^optimizer/gradients_2/optimizer/main_graph_0/hidden_0/Mul_grad/ReshapeI^optimizer/gradients_2/optimizer/main_graph_0/hidden_0/Mul_grad/Reshape_1
�
Woptimizer/gradients_2/optimizer/main_graph_0/hidden_0/Mul_grad/tuple/control_dependencyIdentityFoptimizer/gradients_2/optimizer/main_graph_0/hidden_0/Mul_grad/ReshapeP^optimizer/gradients_2/optimizer/main_graph_0/hidden_0/Mul_grad/tuple/group_deps*
T0*Y
_classO
MKloc:@optimizer/gradients_2/optimizer/main_graph_0/hidden_0/Mul_grad/Reshape
�
Yoptimizer/gradients_2/optimizer/main_graph_0/hidden_0/Mul_grad/tuple/control_dependency_1IdentityHoptimizer/gradients_2/optimizer/main_graph_0/hidden_0/Mul_grad/Reshape_1P^optimizer/gradients_2/optimizer/main_graph_0/hidden_0/Mul_grad/tuple/group_deps*
T0*[
_classQ
OMloc:@optimizer/gradients_2/optimizer/main_graph_0/hidden_0/Mul_grad/Reshape_1
a
1optimizer/gradients_2/policy_1/truediv_grad/ShapeShapepolicy_1/Mul*
T0*
out_type0
c
3optimizer/gradients_2/policy_1/truediv_grad/Shape_1Shapepolicy_1/Sum*
T0*
out_type0
�
Aoptimizer/gradients_2/policy_1/truediv_grad/BroadcastGradientArgsBroadcastGradientArgs1optimizer/gradients_2/policy_1/truediv_grad/Shape3optimizer/gradients_2/policy_1/truediv_grad/Shape_1*
T0
�
3optimizer/gradients_2/policy_1/truediv_grad/RealDivRealDivBoptimizer/gradients_2/policy_1/add_4_grad/tuple/control_dependencypolicy_1/Sum*
T0
�
/optimizer/gradients_2/policy_1/truediv_grad/SumSum3optimizer/gradients_2/policy_1/truediv_grad/RealDivAoptimizer/gradients_2/policy_1/truediv_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
3optimizer/gradients_2/policy_1/truediv_grad/ReshapeReshape/optimizer/gradients_2/policy_1/truediv_grad/Sum1optimizer/gradients_2/policy_1/truediv_grad/Shape*
T0*
Tshape0
M
/optimizer/gradients_2/policy_1/truediv_grad/NegNegpolicy_1/Mul*
T0
�
5optimizer/gradients_2/policy_1/truediv_grad/RealDiv_1RealDiv/optimizer/gradients_2/policy_1/truediv_grad/Negpolicy_1/Sum*
T0
�
5optimizer/gradients_2/policy_1/truediv_grad/RealDiv_2RealDiv5optimizer/gradients_2/policy_1/truediv_grad/RealDiv_1policy_1/Sum*
T0
�
/optimizer/gradients_2/policy_1/truediv_grad/mulMulBoptimizer/gradients_2/policy_1/add_4_grad/tuple/control_dependency5optimizer/gradients_2/policy_1/truediv_grad/RealDiv_2*
T0
�
1optimizer/gradients_2/policy_1/truediv_grad/Sum_1Sum/optimizer/gradients_2/policy_1/truediv_grad/mulCoptimizer/gradients_2/policy_1/truediv_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
5optimizer/gradients_2/policy_1/truediv_grad/Reshape_1Reshape1optimizer/gradients_2/policy_1/truediv_grad/Sum_13optimizer/gradients_2/policy_1/truediv_grad/Shape_1*
T0*
Tshape0
�
<optimizer/gradients_2/policy_1/truediv_grad/tuple/group_depsNoOp4^optimizer/gradients_2/policy_1/truediv_grad/Reshape6^optimizer/gradients_2/policy_1/truediv_grad/Reshape_1
�
Doptimizer/gradients_2/policy_1/truediv_grad/tuple/control_dependencyIdentity3optimizer/gradients_2/policy_1/truediv_grad/Reshape=^optimizer/gradients_2/policy_1/truediv_grad/tuple/group_deps*
T0*F
_class<
:8loc:@optimizer/gradients_2/policy_1/truediv_grad/Reshape
�
Foptimizer/gradients_2/policy_1/truediv_grad/tuple/control_dependency_1Identity5optimizer/gradients_2/policy_1/truediv_grad/Reshape_1=^optimizer/gradients_2/policy_1/truediv_grad/tuple/group_deps*
T0*H
_class>
<:loc:@optimizer/gradients_2/policy_1/truediv_grad/Reshape_1
e
3optimizer/gradients_2/policy_1/truediv_1_grad/ShapeShapepolicy_1/Mul_1*
T0*
out_type0
g
5optimizer/gradients_2/policy_1/truediv_1_grad/Shape_1Shapepolicy_1/Sum_1*
T0*
out_type0
�
Coptimizer/gradients_2/policy_1/truediv_1_grad/BroadcastGradientArgsBroadcastGradientArgs3optimizer/gradients_2/policy_1/truediv_1_grad/Shape5optimizer/gradients_2/policy_1/truediv_1_grad/Shape_1*
T0
�
5optimizer/gradients_2/policy_1/truediv_1_grad/RealDivRealDivBoptimizer/gradients_2/policy_1/add_5_grad/tuple/control_dependencypolicy_1/Sum_1*
T0
�
1optimizer/gradients_2/policy_1/truediv_1_grad/SumSum5optimizer/gradients_2/policy_1/truediv_1_grad/RealDivCoptimizer/gradients_2/policy_1/truediv_1_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
5optimizer/gradients_2/policy_1/truediv_1_grad/ReshapeReshape1optimizer/gradients_2/policy_1/truediv_1_grad/Sum3optimizer/gradients_2/policy_1/truediv_1_grad/Shape*
T0*
Tshape0
Q
1optimizer/gradients_2/policy_1/truediv_1_grad/NegNegpolicy_1/Mul_1*
T0
�
7optimizer/gradients_2/policy_1/truediv_1_grad/RealDiv_1RealDiv1optimizer/gradients_2/policy_1/truediv_1_grad/Negpolicy_1/Sum_1*
T0
�
7optimizer/gradients_2/policy_1/truediv_1_grad/RealDiv_2RealDiv7optimizer/gradients_2/policy_1/truediv_1_grad/RealDiv_1policy_1/Sum_1*
T0
�
1optimizer/gradients_2/policy_1/truediv_1_grad/mulMulBoptimizer/gradients_2/policy_1/add_5_grad/tuple/control_dependency7optimizer/gradients_2/policy_1/truediv_1_grad/RealDiv_2*
T0
�
3optimizer/gradients_2/policy_1/truediv_1_grad/Sum_1Sum1optimizer/gradients_2/policy_1/truediv_1_grad/mulEoptimizer/gradients_2/policy_1/truediv_1_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
7optimizer/gradients_2/policy_1/truediv_1_grad/Reshape_1Reshape3optimizer/gradients_2/policy_1/truediv_1_grad/Sum_15optimizer/gradients_2/policy_1/truediv_1_grad/Shape_1*
T0*
Tshape0
�
>optimizer/gradients_2/policy_1/truediv_1_grad/tuple/group_depsNoOp6^optimizer/gradients_2/policy_1/truediv_1_grad/Reshape8^optimizer/gradients_2/policy_1/truediv_1_grad/Reshape_1
�
Foptimizer/gradients_2/policy_1/truediv_1_grad/tuple/control_dependencyIdentity5optimizer/gradients_2/policy_1/truediv_1_grad/Reshape?^optimizer/gradients_2/policy_1/truediv_1_grad/tuple/group_deps*
T0*H
_class>
<:loc:@optimizer/gradients_2/policy_1/truediv_1_grad/Reshape
�
Hoptimizer/gradients_2/policy_1/truediv_1_grad/tuple/control_dependency_1Identity7optimizer/gradients_2/policy_1/truediv_1_grad/Reshape_1?^optimizer/gradients_2/policy_1/truediv_1_grad/tuple/group_deps*
T0*J
_class@
><loc:@optimizer/gradients_2/policy_1/truediv_1_grad/Reshape_1
�
Noptimizer/gradients_2/optimizer/main_graph_0/hidden_0/Sigmoid_grad/SigmoidGradSigmoidGrad'optimizer/main_graph_0/hidden_0/SigmoidYoptimizer/gradients_2/optimizer/main_graph_0/hidden_0/Mul_grad/tuple/control_dependency_1*
T0
]
-optimizer/gradients_2/policy_1/Sum_grad/ShapeShapepolicy_1/Mul*
T0*
out_type0
�
3optimizer/gradients_2/policy_1/Sum_grad/BroadcastToBroadcastToFoptimizer/gradients_2/policy_1/truediv_grad/tuple/control_dependency_1-optimizer/gradients_2/policy_1/Sum_grad/Shape*
T0*

Tidx0
a
/optimizer/gradients_2/policy_1/Sum_1_grad/ShapeShapepolicy_1/Mul_1*
T0*
out_type0
�
5optimizer/gradients_2/policy_1/Sum_1_grad/BroadcastToBroadcastToHoptimizer/gradients_2/policy_1/truediv_1_grad/tuple/control_dependency_1/optimizer/gradients_2/policy_1/Sum_1_grad/Shape*
T0*

Tidx0
�
optimizer/gradients_2/AddN_6AddNWoptimizer/gradients_2/optimizer/main_graph_0/hidden_0/Mul_grad/tuple/control_dependencyNoptimizer/gradients_2/optimizer/main_graph_0/hidden_0/Sigmoid_grad/SigmoidGrad*
N*
T0*Y
_classO
MKloc:@optimizer/gradients_2/optimizer/main_graph_0/hidden_0/Mul_grad/Reshape
�
Noptimizer/gradients_2/optimizer/main_graph_0/hidden_0/BiasAdd_grad/BiasAddGradBiasAddGradoptimizer/gradients_2/AddN_6*
T0*
data_formatNHWC
�
Soptimizer/gradients_2/optimizer/main_graph_0/hidden_0/BiasAdd_grad/tuple/group_depsNoOp^optimizer/gradients_2/AddN_6O^optimizer/gradients_2/optimizer/main_graph_0/hidden_0/BiasAdd_grad/BiasAddGrad
�
[optimizer/gradients_2/optimizer/main_graph_0/hidden_0/BiasAdd_grad/tuple/control_dependencyIdentityoptimizer/gradients_2/AddN_6T^optimizer/gradients_2/optimizer/main_graph_0/hidden_0/BiasAdd_grad/tuple/group_deps*
T0*Y
_classO
MKloc:@optimizer/gradients_2/optimizer/main_graph_0/hidden_0/Mul_grad/Reshape
�
]optimizer/gradients_2/optimizer/main_graph_0/hidden_0/BiasAdd_grad/tuple/control_dependency_1IdentityNoptimizer/gradients_2/optimizer/main_graph_0/hidden_0/BiasAdd_grad/BiasAddGradT^optimizer/gradients_2/optimizer/main_graph_0/hidden_0/BiasAdd_grad/tuple/group_deps*
T0*a
_classW
USloc:@optimizer/gradients_2/optimizer/main_graph_0/hidden_0/BiasAdd_grad/BiasAddGrad
�
optimizer/gradients_2/AddN_7AddNDoptimizer/gradients_2/policy_1/truediv_grad/tuple/control_dependency3optimizer/gradients_2/policy_1/Sum_grad/BroadcastTo*
N*
T0*F
_class<
:8loc:@optimizer/gradients_2/policy_1/truediv_grad/Reshape
]
-optimizer/gradients_2/policy_1/Mul_grad/ShapeShapepolicy_1/add*
T0*
out_type0
i
/optimizer/gradients_2/policy_1/Mul_grad/Shape_1Shapepolicy_1/strided_slice*
T0*
out_type0
�
=optimizer/gradients_2/policy_1/Mul_grad/BroadcastGradientArgsBroadcastGradientArgs-optimizer/gradients_2/policy_1/Mul_grad/Shape/optimizer/gradients_2/policy_1/Mul_grad/Shape_1*
T0
q
+optimizer/gradients_2/policy_1/Mul_grad/MulMuloptimizer/gradients_2/AddN_7policy_1/strided_slice*
T0
�
+optimizer/gradients_2/policy_1/Mul_grad/SumSum+optimizer/gradients_2/policy_1/Mul_grad/Mul=optimizer/gradients_2/policy_1/Mul_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
/optimizer/gradients_2/policy_1/Mul_grad/ReshapeReshape+optimizer/gradients_2/policy_1/Mul_grad/Sum-optimizer/gradients_2/policy_1/Mul_grad/Shape*
T0*
Tshape0
i
-optimizer/gradients_2/policy_1/Mul_grad/Mul_1Mulpolicy_1/addoptimizer/gradients_2/AddN_7*
T0
�
-optimizer/gradients_2/policy_1/Mul_grad/Sum_1Sum-optimizer/gradients_2/policy_1/Mul_grad/Mul_1?optimizer/gradients_2/policy_1/Mul_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
1optimizer/gradients_2/policy_1/Mul_grad/Reshape_1Reshape-optimizer/gradients_2/policy_1/Mul_grad/Sum_1/optimizer/gradients_2/policy_1/Mul_grad/Shape_1*
T0*
Tshape0
�
8optimizer/gradients_2/policy_1/Mul_grad/tuple/group_depsNoOp0^optimizer/gradients_2/policy_1/Mul_grad/Reshape2^optimizer/gradients_2/policy_1/Mul_grad/Reshape_1
�
@optimizer/gradients_2/policy_1/Mul_grad/tuple/control_dependencyIdentity/optimizer/gradients_2/policy_1/Mul_grad/Reshape9^optimizer/gradients_2/policy_1/Mul_grad/tuple/group_deps*
T0*B
_class8
64loc:@optimizer/gradients_2/policy_1/Mul_grad/Reshape
�
Boptimizer/gradients_2/policy_1/Mul_grad/tuple/control_dependency_1Identity1optimizer/gradients_2/policy_1/Mul_grad/Reshape_19^optimizer/gradients_2/policy_1/Mul_grad/tuple/group_deps*
T0*D
_class:
86loc:@optimizer/gradients_2/policy_1/Mul_grad/Reshape_1
�
optimizer/gradients_2/AddN_8AddNFoptimizer/gradients_2/policy_1/truediv_1_grad/tuple/control_dependency5optimizer/gradients_2/policy_1/Sum_1_grad/BroadcastTo*
N*
T0*H
_class>
<:loc:@optimizer/gradients_2/policy_1/truediv_1_grad/Reshape
a
/optimizer/gradients_2/policy_1/Mul_1_grad/ShapeShapepolicy_1/add_1*
T0*
out_type0
m
1optimizer/gradients_2/policy_1/Mul_1_grad/Shape_1Shapepolicy_1/strided_slice_1*
T0*
out_type0
�
?optimizer/gradients_2/policy_1/Mul_1_grad/BroadcastGradientArgsBroadcastGradientArgs/optimizer/gradients_2/policy_1/Mul_1_grad/Shape1optimizer/gradients_2/policy_1/Mul_1_grad/Shape_1*
T0
u
-optimizer/gradients_2/policy_1/Mul_1_grad/MulMuloptimizer/gradients_2/AddN_8policy_1/strided_slice_1*
T0
�
-optimizer/gradients_2/policy_1/Mul_1_grad/SumSum-optimizer/gradients_2/policy_1/Mul_1_grad/Mul?optimizer/gradients_2/policy_1/Mul_1_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
1optimizer/gradients_2/policy_1/Mul_1_grad/ReshapeReshape-optimizer/gradients_2/policy_1/Mul_1_grad/Sum/optimizer/gradients_2/policy_1/Mul_1_grad/Shape*
T0*
Tshape0
m
/optimizer/gradients_2/policy_1/Mul_1_grad/Mul_1Mulpolicy_1/add_1optimizer/gradients_2/AddN_8*
T0
�
/optimizer/gradients_2/policy_1/Mul_1_grad/Sum_1Sum/optimizer/gradients_2/policy_1/Mul_1_grad/Mul_1Aoptimizer/gradients_2/policy_1/Mul_1_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
3optimizer/gradients_2/policy_1/Mul_1_grad/Reshape_1Reshape/optimizer/gradients_2/policy_1/Mul_1_grad/Sum_11optimizer/gradients_2/policy_1/Mul_1_grad/Shape_1*
T0*
Tshape0
�
:optimizer/gradients_2/policy_1/Mul_1_grad/tuple/group_depsNoOp2^optimizer/gradients_2/policy_1/Mul_1_grad/Reshape4^optimizer/gradients_2/policy_1/Mul_1_grad/Reshape_1
�
Boptimizer/gradients_2/policy_1/Mul_1_grad/tuple/control_dependencyIdentity1optimizer/gradients_2/policy_1/Mul_1_grad/Reshape;^optimizer/gradients_2/policy_1/Mul_1_grad/tuple/group_deps*
T0*D
_class:
86loc:@optimizer/gradients_2/policy_1/Mul_1_grad/Reshape
�
Doptimizer/gradients_2/policy_1/Mul_1_grad/tuple/control_dependency_1Identity3optimizer/gradients_2/policy_1/Mul_1_grad/Reshape_1;^optimizer/gradients_2/policy_1/Mul_1_grad/tuple/group_deps*
T0*F
_class<
:8loc:@optimizer/gradients_2/policy_1/Mul_1_grad/Reshape_1
�
Hoptimizer/gradients_2/optimizer/main_graph_0/hidden_0/MatMul_grad/MatMulMatMul[optimizer/gradients_2/optimizer/main_graph_0/hidden_0/BiasAdd_grad/tuple/control_dependency,optimizer//main_graph_0/hidden_0/kernel/read*
T0*
transpose_a( *
transpose_b(
�
Joptimizer/gradients_2/optimizer/main_graph_0/hidden_0/MatMul_grad/MatMul_1MatMulvector_observation[optimizer/gradients_2/optimizer/main_graph_0/hidden_0/BiasAdd_grad/tuple/control_dependency*
T0*
transpose_a(*
transpose_b( 
�
Roptimizer/gradients_2/optimizer/main_graph_0/hidden_0/MatMul_grad/tuple/group_depsNoOpI^optimizer/gradients_2/optimizer/main_graph_0/hidden_0/MatMul_grad/MatMulK^optimizer/gradients_2/optimizer/main_graph_0/hidden_0/MatMul_grad/MatMul_1
�
Zoptimizer/gradients_2/optimizer/main_graph_0/hidden_0/MatMul_grad/tuple/control_dependencyIdentityHoptimizer/gradients_2/optimizer/main_graph_0/hidden_0/MatMul_grad/MatMulS^optimizer/gradients_2/optimizer/main_graph_0/hidden_0/MatMul_grad/tuple/group_deps*
T0*[
_classQ
OMloc:@optimizer/gradients_2/optimizer/main_graph_0/hidden_0/MatMul_grad/MatMul
�
\optimizer/gradients_2/optimizer/main_graph_0/hidden_0/MatMul_grad/tuple/control_dependency_1IdentityJoptimizer/gradients_2/optimizer/main_graph_0/hidden_0/MatMul_grad/MatMul_1S^optimizer/gradients_2/optimizer/main_graph_0/hidden_0/MatMul_grad/tuple/group_deps*
T0*]
_classS
QOloc:@optimizer/gradients_2/optimizer/main_graph_0/hidden_0/MatMul_grad/MatMul_1
a
-optimizer/gradients_2/policy_1/add_grad/ShapeShapepolicy_1/Softmax*
T0*
out_type0
a
/optimizer/gradients_2/policy_1/add_grad/Shape_1Shapepolicy_1/add/y*
T0*
out_type0
�
=optimizer/gradients_2/policy_1/add_grad/BroadcastGradientArgsBroadcastGradientArgs-optimizer/gradients_2/policy_1/add_grad/Shape/optimizer/gradients_2/policy_1/add_grad/Shape_1*
T0
�
+optimizer/gradients_2/policy_1/add_grad/SumSum@optimizer/gradients_2/policy_1/Mul_grad/tuple/control_dependency=optimizer/gradients_2/policy_1/add_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
/optimizer/gradients_2/policy_1/add_grad/ReshapeReshape+optimizer/gradients_2/policy_1/add_grad/Sum-optimizer/gradients_2/policy_1/add_grad/Shape*
T0*
Tshape0
�
-optimizer/gradients_2/policy_1/add_grad/Sum_1Sum@optimizer/gradients_2/policy_1/Mul_grad/tuple/control_dependency?optimizer/gradients_2/policy_1/add_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
1optimizer/gradients_2/policy_1/add_grad/Reshape_1Reshape-optimizer/gradients_2/policy_1/add_grad/Sum_1/optimizer/gradients_2/policy_1/add_grad/Shape_1*
T0*
Tshape0
�
8optimizer/gradients_2/policy_1/add_grad/tuple/group_depsNoOp0^optimizer/gradients_2/policy_1/add_grad/Reshape2^optimizer/gradients_2/policy_1/add_grad/Reshape_1
�
@optimizer/gradients_2/policy_1/add_grad/tuple/control_dependencyIdentity/optimizer/gradients_2/policy_1/add_grad/Reshape9^optimizer/gradients_2/policy_1/add_grad/tuple/group_deps*
T0*B
_class8
64loc:@optimizer/gradients_2/policy_1/add_grad/Reshape
�
Boptimizer/gradients_2/policy_1/add_grad/tuple/control_dependency_1Identity1optimizer/gradients_2/policy_1/add_grad/Reshape_19^optimizer/gradients_2/policy_1/add_grad/tuple/group_deps*
T0*D
_class:
86loc:@optimizer/gradients_2/policy_1/add_grad/Reshape_1
e
/optimizer/gradients_2/policy_1/add_1_grad/ShapeShapepolicy_1/Softmax_1*
T0*
out_type0
e
1optimizer/gradients_2/policy_1/add_1_grad/Shape_1Shapepolicy_1/add_1/y*
T0*
out_type0
�
?optimizer/gradients_2/policy_1/add_1_grad/BroadcastGradientArgsBroadcastGradientArgs/optimizer/gradients_2/policy_1/add_1_grad/Shape1optimizer/gradients_2/policy_1/add_1_grad/Shape_1*
T0
�
-optimizer/gradients_2/policy_1/add_1_grad/SumSumBoptimizer/gradients_2/policy_1/Mul_1_grad/tuple/control_dependency?optimizer/gradients_2/policy_1/add_1_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
1optimizer/gradients_2/policy_1/add_1_grad/ReshapeReshape-optimizer/gradients_2/policy_1/add_1_grad/Sum/optimizer/gradients_2/policy_1/add_1_grad/Shape*
T0*
Tshape0
�
/optimizer/gradients_2/policy_1/add_1_grad/Sum_1SumBoptimizer/gradients_2/policy_1/Mul_1_grad/tuple/control_dependencyAoptimizer/gradients_2/policy_1/add_1_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
3optimizer/gradients_2/policy_1/add_1_grad/Reshape_1Reshape/optimizer/gradients_2/policy_1/add_1_grad/Sum_11optimizer/gradients_2/policy_1/add_1_grad/Shape_1*
T0*
Tshape0
�
:optimizer/gradients_2/policy_1/add_1_grad/tuple/group_depsNoOp2^optimizer/gradients_2/policy_1/add_1_grad/Reshape4^optimizer/gradients_2/policy_1/add_1_grad/Reshape_1
�
Boptimizer/gradients_2/policy_1/add_1_grad/tuple/control_dependencyIdentity1optimizer/gradients_2/policy_1/add_1_grad/Reshape;^optimizer/gradients_2/policy_1/add_1_grad/tuple/group_deps*
T0*D
_class:
86loc:@optimizer/gradients_2/policy_1/add_1_grad/Reshape
�
Doptimizer/gradients_2/policy_1/add_1_grad/tuple/control_dependency_1Identity3optimizer/gradients_2/policy_1/add_1_grad/Reshape_1;^optimizer/gradients_2/policy_1/add_1_grad/tuple/group_deps*
T0*F
_class<
:8loc:@optimizer/gradients_2/policy_1/add_1_grad/Reshape_1
�
/optimizer/gradients_2/policy_1/Softmax_grad/mulMul@optimizer/gradients_2/policy_1/add_grad/tuple/control_dependencypolicy_1/Softmax*
T0
t
Aoptimizer/gradients_2/policy_1/Softmax_grad/Sum/reduction_indicesConst*
dtype0*
valueB :
���������
�
/optimizer/gradients_2/policy_1/Softmax_grad/SumSum/optimizer/gradients_2/policy_1/Softmax_grad/mulAoptimizer/gradients_2/policy_1/Softmax_grad/Sum/reduction_indices*
T0*

Tidx0*
	keep_dims(
�
/optimizer/gradients_2/policy_1/Softmax_grad/subSub@optimizer/gradients_2/policy_1/add_grad/tuple/control_dependency/optimizer/gradients_2/policy_1/Softmax_grad/Sum*
T0
�
1optimizer/gradients_2/policy_1/Softmax_grad/mul_1Mul/optimizer/gradients_2/policy_1/Softmax_grad/subpolicy_1/Softmax*
T0
�
1optimizer/gradients_2/policy_1/Softmax_1_grad/mulMulBoptimizer/gradients_2/policy_1/add_1_grad/tuple/control_dependencypolicy_1/Softmax_1*
T0
v
Coptimizer/gradients_2/policy_1/Softmax_1_grad/Sum/reduction_indicesConst*
dtype0*
valueB :
���������
�
1optimizer/gradients_2/policy_1/Softmax_1_grad/SumSum1optimizer/gradients_2/policy_1/Softmax_1_grad/mulCoptimizer/gradients_2/policy_1/Softmax_1_grad/Sum/reduction_indices*
T0*

Tidx0*
	keep_dims(
�
1optimizer/gradients_2/policy_1/Softmax_1_grad/subSubBoptimizer/gradients_2/policy_1/add_1_grad/tuple/control_dependency1optimizer/gradients_2/policy_1/Softmax_1_grad/Sum*
T0
�
3optimizer/gradients_2/policy_1/Softmax_1_grad/mul_1Mul1optimizer/gradients_2/policy_1/Softmax_1_grad/subpolicy_1/Softmax_1*
T0
�
7optimizer/gradients_2/policy_1/dense/MatMul_grad/MatMulMatMul1optimizer/gradients_2/policy_1/Softmax_grad/mul_1policy/dense/kernel/read*
T0*
transpose_a( *
transpose_b(
�
9optimizer/gradients_2/policy_1/dense/MatMul_grad/MatMul_1MatMul policy/main_graph_0/hidden_1/Mul1optimizer/gradients_2/policy_1/Softmax_grad/mul_1*
T0*
transpose_a(*
transpose_b( 
�
Aoptimizer/gradients_2/policy_1/dense/MatMul_grad/tuple/group_depsNoOp8^optimizer/gradients_2/policy_1/dense/MatMul_grad/MatMul:^optimizer/gradients_2/policy_1/dense/MatMul_grad/MatMul_1
�
Ioptimizer/gradients_2/policy_1/dense/MatMul_grad/tuple/control_dependencyIdentity7optimizer/gradients_2/policy_1/dense/MatMul_grad/MatMulB^optimizer/gradients_2/policy_1/dense/MatMul_grad/tuple/group_deps*
T0*J
_class@
><loc:@optimizer/gradients_2/policy_1/dense/MatMul_grad/MatMul
�
Koptimizer/gradients_2/policy_1/dense/MatMul_grad/tuple/control_dependency_1Identity9optimizer/gradients_2/policy_1/dense/MatMul_grad/MatMul_1B^optimizer/gradients_2/policy_1/dense/MatMul_grad/tuple/group_deps*
T0*L
_classB
@>loc:@optimizer/gradients_2/policy_1/dense/MatMul_grad/MatMul_1
�
9optimizer/gradients_2/policy_1/dense_1/MatMul_grad/MatMulMatMul3optimizer/gradients_2/policy_1/Softmax_1_grad/mul_1policy/dense_1/kernel/read*
T0*
transpose_a( *
transpose_b(
�
;optimizer/gradients_2/policy_1/dense_1/MatMul_grad/MatMul_1MatMul policy/main_graph_0/hidden_1/Mul3optimizer/gradients_2/policy_1/Softmax_1_grad/mul_1*
T0*
transpose_a(*
transpose_b( 
�
Coptimizer/gradients_2/policy_1/dense_1/MatMul_grad/tuple/group_depsNoOp:^optimizer/gradients_2/policy_1/dense_1/MatMul_grad/MatMul<^optimizer/gradients_2/policy_1/dense_1/MatMul_grad/MatMul_1
�
Koptimizer/gradients_2/policy_1/dense_1/MatMul_grad/tuple/control_dependencyIdentity9optimizer/gradients_2/policy_1/dense_1/MatMul_grad/MatMulD^optimizer/gradients_2/policy_1/dense_1/MatMul_grad/tuple/group_deps*
T0*L
_classB
@>loc:@optimizer/gradients_2/policy_1/dense_1/MatMul_grad/MatMul
�
Moptimizer/gradients_2/policy_1/dense_1/MatMul_grad/tuple/control_dependency_1Identity;optimizer/gradients_2/policy_1/dense_1/MatMul_grad/MatMul_1D^optimizer/gradients_2/policy_1/dense_1/MatMul_grad/tuple/group_deps*
T0*N
_classD
B@loc:@optimizer/gradients_2/policy_1/dense_1/MatMul_grad/MatMul_1
�
optimizer/gradients_2/AddN_9AddNIoptimizer/gradients_2/policy_1/dense/MatMul_grad/tuple/control_dependencyKoptimizer/gradients_2/policy_1/dense_1/MatMul_grad/tuple/control_dependency*
N*
T0*J
_class@
><loc:@optimizer/gradients_2/policy_1/dense/MatMul_grad/MatMul
�
Aoptimizer/gradients_2/policy/main_graph_0/hidden_1/Mul_grad/ShapeShape$policy/main_graph_0/hidden_1/BiasAdd*
T0*
out_type0
�
Coptimizer/gradients_2/policy/main_graph_0/hidden_1/Mul_grad/Shape_1Shape$policy/main_graph_0/hidden_1/Sigmoid*
T0*
out_type0
�
Qoptimizer/gradients_2/policy/main_graph_0/hidden_1/Mul_grad/BroadcastGradientArgsBroadcastGradientArgsAoptimizer/gradients_2/policy/main_graph_0/hidden_1/Mul_grad/ShapeCoptimizer/gradients_2/policy/main_graph_0/hidden_1/Mul_grad/Shape_1*
T0
�
?optimizer/gradients_2/policy/main_graph_0/hidden_1/Mul_grad/MulMuloptimizer/gradients_2/AddN_9$policy/main_graph_0/hidden_1/Sigmoid*
T0
�
?optimizer/gradients_2/policy/main_graph_0/hidden_1/Mul_grad/SumSum?optimizer/gradients_2/policy/main_graph_0/hidden_1/Mul_grad/MulQoptimizer/gradients_2/policy/main_graph_0/hidden_1/Mul_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
Coptimizer/gradients_2/policy/main_graph_0/hidden_1/Mul_grad/ReshapeReshape?optimizer/gradients_2/policy/main_graph_0/hidden_1/Mul_grad/SumAoptimizer/gradients_2/policy/main_graph_0/hidden_1/Mul_grad/Shape*
T0*
Tshape0
�
Aoptimizer/gradients_2/policy/main_graph_0/hidden_1/Mul_grad/Mul_1Mul$policy/main_graph_0/hidden_1/BiasAddoptimizer/gradients_2/AddN_9*
T0
�
Aoptimizer/gradients_2/policy/main_graph_0/hidden_1/Mul_grad/Sum_1SumAoptimizer/gradients_2/policy/main_graph_0/hidden_1/Mul_grad/Mul_1Soptimizer/gradients_2/policy/main_graph_0/hidden_1/Mul_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
Eoptimizer/gradients_2/policy/main_graph_0/hidden_1/Mul_grad/Reshape_1ReshapeAoptimizer/gradients_2/policy/main_graph_0/hidden_1/Mul_grad/Sum_1Coptimizer/gradients_2/policy/main_graph_0/hidden_1/Mul_grad/Shape_1*
T0*
Tshape0
�
Loptimizer/gradients_2/policy/main_graph_0/hidden_1/Mul_grad/tuple/group_depsNoOpD^optimizer/gradients_2/policy/main_graph_0/hidden_1/Mul_grad/ReshapeF^optimizer/gradients_2/policy/main_graph_0/hidden_1/Mul_grad/Reshape_1
�
Toptimizer/gradients_2/policy/main_graph_0/hidden_1/Mul_grad/tuple/control_dependencyIdentityCoptimizer/gradients_2/policy/main_graph_0/hidden_1/Mul_grad/ReshapeM^optimizer/gradients_2/policy/main_graph_0/hidden_1/Mul_grad/tuple/group_deps*
T0*V
_classL
JHloc:@optimizer/gradients_2/policy/main_graph_0/hidden_1/Mul_grad/Reshape
�
Voptimizer/gradients_2/policy/main_graph_0/hidden_1/Mul_grad/tuple/control_dependency_1IdentityEoptimizer/gradients_2/policy/main_graph_0/hidden_1/Mul_grad/Reshape_1M^optimizer/gradients_2/policy/main_graph_0/hidden_1/Mul_grad/tuple/group_deps*
T0*X
_classN
LJloc:@optimizer/gradients_2/policy/main_graph_0/hidden_1/Mul_grad/Reshape_1
�
Koptimizer/gradients_2/policy/main_graph_0/hidden_1/Sigmoid_grad/SigmoidGradSigmoidGrad$policy/main_graph_0/hidden_1/SigmoidVoptimizer/gradients_2/policy/main_graph_0/hidden_1/Mul_grad/tuple/control_dependency_1*
T0
�
optimizer/gradients_2/AddN_10AddNToptimizer/gradients_2/policy/main_graph_0/hidden_1/Mul_grad/tuple/control_dependencyKoptimizer/gradients_2/policy/main_graph_0/hidden_1/Sigmoid_grad/SigmoidGrad*
N*
T0*V
_classL
JHloc:@optimizer/gradients_2/policy/main_graph_0/hidden_1/Mul_grad/Reshape
�
Koptimizer/gradients_2/policy/main_graph_0/hidden_1/BiasAdd_grad/BiasAddGradBiasAddGradoptimizer/gradients_2/AddN_10*
T0*
data_formatNHWC
�
Poptimizer/gradients_2/policy/main_graph_0/hidden_1/BiasAdd_grad/tuple/group_depsNoOp^optimizer/gradients_2/AddN_10L^optimizer/gradients_2/policy/main_graph_0/hidden_1/BiasAdd_grad/BiasAddGrad
�
Xoptimizer/gradients_2/policy/main_graph_0/hidden_1/BiasAdd_grad/tuple/control_dependencyIdentityoptimizer/gradients_2/AddN_10Q^optimizer/gradients_2/policy/main_graph_0/hidden_1/BiasAdd_grad/tuple/group_deps*
T0*V
_classL
JHloc:@optimizer/gradients_2/policy/main_graph_0/hidden_1/Mul_grad/Reshape
�
Zoptimizer/gradients_2/policy/main_graph_0/hidden_1/BiasAdd_grad/tuple/control_dependency_1IdentityKoptimizer/gradients_2/policy/main_graph_0/hidden_1/BiasAdd_grad/BiasAddGradQ^optimizer/gradients_2/policy/main_graph_0/hidden_1/BiasAdd_grad/tuple/group_deps*
T0*^
_classT
RPloc:@optimizer/gradients_2/policy/main_graph_0/hidden_1/BiasAdd_grad/BiasAddGrad
�
Eoptimizer/gradients_2/policy/main_graph_0/hidden_1/MatMul_grad/MatMulMatMulXoptimizer/gradients_2/policy/main_graph_0/hidden_1/BiasAdd_grad/tuple/control_dependency(policy/main_graph_0/hidden_1/kernel/read*
T0*
transpose_a( *
transpose_b(
�
Goptimizer/gradients_2/policy/main_graph_0/hidden_1/MatMul_grad/MatMul_1MatMul policy/main_graph_0/hidden_0/MulXoptimizer/gradients_2/policy/main_graph_0/hidden_1/BiasAdd_grad/tuple/control_dependency*
T0*
transpose_a(*
transpose_b( 
�
Ooptimizer/gradients_2/policy/main_graph_0/hidden_1/MatMul_grad/tuple/group_depsNoOpF^optimizer/gradients_2/policy/main_graph_0/hidden_1/MatMul_grad/MatMulH^optimizer/gradients_2/policy/main_graph_0/hidden_1/MatMul_grad/MatMul_1
�
Woptimizer/gradients_2/policy/main_graph_0/hidden_1/MatMul_grad/tuple/control_dependencyIdentityEoptimizer/gradients_2/policy/main_graph_0/hidden_1/MatMul_grad/MatMulP^optimizer/gradients_2/policy/main_graph_0/hidden_1/MatMul_grad/tuple/group_deps*
T0*X
_classN
LJloc:@optimizer/gradients_2/policy/main_graph_0/hidden_1/MatMul_grad/MatMul
�
Yoptimizer/gradients_2/policy/main_graph_0/hidden_1/MatMul_grad/tuple/control_dependency_1IdentityGoptimizer/gradients_2/policy/main_graph_0/hidden_1/MatMul_grad/MatMul_1P^optimizer/gradients_2/policy/main_graph_0/hidden_1/MatMul_grad/tuple/group_deps*
T0*Z
_classP
NLloc:@optimizer/gradients_2/policy/main_graph_0/hidden_1/MatMul_grad/MatMul_1
�
Aoptimizer/gradients_2/policy/main_graph_0/hidden_0/Mul_grad/ShapeShape$policy/main_graph_0/hidden_0/BiasAdd*
T0*
out_type0
�
Coptimizer/gradients_2/policy/main_graph_0/hidden_0/Mul_grad/Shape_1Shape$policy/main_graph_0/hidden_0/Sigmoid*
T0*
out_type0
�
Qoptimizer/gradients_2/policy/main_graph_0/hidden_0/Mul_grad/BroadcastGradientArgsBroadcastGradientArgsAoptimizer/gradients_2/policy/main_graph_0/hidden_0/Mul_grad/ShapeCoptimizer/gradients_2/policy/main_graph_0/hidden_0/Mul_grad/Shape_1*
T0
�
?optimizer/gradients_2/policy/main_graph_0/hidden_0/Mul_grad/MulMulWoptimizer/gradients_2/policy/main_graph_0/hidden_1/MatMul_grad/tuple/control_dependency$policy/main_graph_0/hidden_0/Sigmoid*
T0
�
?optimizer/gradients_2/policy/main_graph_0/hidden_0/Mul_grad/SumSum?optimizer/gradients_2/policy/main_graph_0/hidden_0/Mul_grad/MulQoptimizer/gradients_2/policy/main_graph_0/hidden_0/Mul_grad/BroadcastGradientArgs*
T0*

Tidx0*
	keep_dims( 
�
Coptimizer/gradients_2/policy/main_graph_0/hidden_0/Mul_grad/ReshapeReshape?optimizer/gradients_2/policy/main_graph_0/hidden_0/Mul_grad/SumAoptimizer/gradients_2/policy/main_graph_0/hidden_0/Mul_grad/Shape*
T0*
Tshape0
�
Aoptimizer/gradients_2/policy/main_graph_0/hidden_0/Mul_grad/Mul_1Mul$policy/main_graph_0/hidden_0/BiasAddWoptimizer/gradients_2/policy/main_graph_0/hidden_1/MatMul_grad/tuple/control_dependency*
T0
�
Aoptimizer/gradients_2/policy/main_graph_0/hidden_0/Mul_grad/Sum_1SumAoptimizer/gradients_2/policy/main_graph_0/hidden_0/Mul_grad/Mul_1Soptimizer/gradients_2/policy/main_graph_0/hidden_0/Mul_grad/BroadcastGradientArgs:1*
T0*

Tidx0*
	keep_dims( 
�
Eoptimizer/gradients_2/policy/main_graph_0/hidden_0/Mul_grad/Reshape_1ReshapeAoptimizer/gradients_2/policy/main_graph_0/hidden_0/Mul_grad/Sum_1Coptimizer/gradients_2/policy/main_graph_0/hidden_0/Mul_grad/Shape_1*
T0*
Tshape0
�
Loptimizer/gradients_2/policy/main_graph_0/hidden_0/Mul_grad/tuple/group_depsNoOpD^optimizer/gradients_2/policy/main_graph_0/hidden_0/Mul_grad/ReshapeF^optimizer/gradients_2/policy/main_graph_0/hidden_0/Mul_grad/Reshape_1
�
Toptimizer/gradients_2/policy/main_graph_0/hidden_0/Mul_grad/tuple/control_dependencyIdentityCoptimizer/gradients_2/policy/main_graph_0/hidden_0/Mul_grad/ReshapeM^optimizer/gradients_2/policy/main_graph_0/hidden_0/Mul_grad/tuple/group_deps*
T0*V
_classL
JHloc:@optimizer/gradients_2/policy/main_graph_0/hidden_0/Mul_grad/Reshape
�
Voptimizer/gradients_2/policy/main_graph_0/hidden_0/Mul_grad/tuple/control_dependency_1IdentityEoptimizer/gradients_2/policy/main_graph_0/hidden_0/Mul_grad/Reshape_1M^optimizer/gradients_2/policy/main_graph_0/hidden_0/Mul_grad/tuple/group_deps*
T0*X
_classN
LJloc:@optimizer/gradients_2/policy/main_graph_0/hidden_0/Mul_grad/Reshape_1
�
Koptimizer/gradients_2/policy/main_graph_0/hidden_0/Sigmoid_grad/SigmoidGradSigmoidGrad$policy/main_graph_0/hidden_0/SigmoidVoptimizer/gradients_2/policy/main_graph_0/hidden_0/Mul_grad/tuple/control_dependency_1*
T0
�
optimizer/gradients_2/AddN_11AddNToptimizer/gradients_2/policy/main_graph_0/hidden_0/Mul_grad/tuple/control_dependencyKoptimizer/gradients_2/policy/main_graph_0/hidden_0/Sigmoid_grad/SigmoidGrad*
N*
T0*V
_classL
JHloc:@optimizer/gradients_2/policy/main_graph_0/hidden_0/Mul_grad/Reshape
�
Koptimizer/gradients_2/policy/main_graph_0/hidden_0/BiasAdd_grad/BiasAddGradBiasAddGradoptimizer/gradients_2/AddN_11*
T0*
data_formatNHWC
�
Poptimizer/gradients_2/policy/main_graph_0/hidden_0/BiasAdd_grad/tuple/group_depsNoOp^optimizer/gradients_2/AddN_11L^optimizer/gradients_2/policy/main_graph_0/hidden_0/BiasAdd_grad/BiasAddGrad
�
Xoptimizer/gradients_2/policy/main_graph_0/hidden_0/BiasAdd_grad/tuple/control_dependencyIdentityoptimizer/gradients_2/AddN_11Q^optimizer/gradients_2/policy/main_graph_0/hidden_0/BiasAdd_grad/tuple/group_deps*
T0*V
_classL
JHloc:@optimizer/gradients_2/policy/main_graph_0/hidden_0/Mul_grad/Reshape
�
Zoptimizer/gradients_2/policy/main_graph_0/hidden_0/BiasAdd_grad/tuple/control_dependency_1IdentityKoptimizer/gradients_2/policy/main_graph_0/hidden_0/BiasAdd_grad/BiasAddGradQ^optimizer/gradients_2/policy/main_graph_0/hidden_0/BiasAdd_grad/tuple/group_deps*
T0*^
_classT
RPloc:@optimizer/gradients_2/policy/main_graph_0/hidden_0/BiasAdd_grad/BiasAddGrad
�
Eoptimizer/gradients_2/policy/main_graph_0/hidden_0/MatMul_grad/MatMulMatMulXoptimizer/gradients_2/policy/main_graph_0/hidden_0/BiasAdd_grad/tuple/control_dependency(policy/main_graph_0/hidden_0/kernel/read*
T0*
transpose_a( *
transpose_b(
�
Goptimizer/gradients_2/policy/main_graph_0/hidden_0/MatMul_grad/MatMul_1MatMulvector_observationXoptimizer/gradients_2/policy/main_graph_0/hidden_0/BiasAdd_grad/tuple/control_dependency*
T0*
transpose_a(*
transpose_b( 
�
Ooptimizer/gradients_2/policy/main_graph_0/hidden_0/MatMul_grad/tuple/group_depsNoOpF^optimizer/gradients_2/policy/main_graph_0/hidden_0/MatMul_grad/MatMulH^optimizer/gradients_2/policy/main_graph_0/hidden_0/MatMul_grad/MatMul_1
�
Woptimizer/gradients_2/policy/main_graph_0/hidden_0/MatMul_grad/tuple/control_dependencyIdentityEoptimizer/gradients_2/policy/main_graph_0/hidden_0/MatMul_grad/MatMulP^optimizer/gradients_2/policy/main_graph_0/hidden_0/MatMul_grad/tuple/group_deps*
T0*X
_classN
LJloc:@optimizer/gradients_2/policy/main_graph_0/hidden_0/MatMul_grad/MatMul
�
Yoptimizer/gradients_2/policy/main_graph_0/hidden_0/MatMul_grad/tuple/control_dependency_1IdentityGoptimizer/gradients_2/policy/main_graph_0/hidden_0/MatMul_grad/MatMul_1P^optimizer/gradients_2/policy/main_graph_0/hidden_0/MatMul_grad/tuple/group_deps*
T0*Z
_classP
NLloc:@optimizer/gradients_2/policy/main_graph_0/hidden_0/MatMul_grad/MatMul_1
�
%optimizer/beta1_power_1/initial_valueConst*2
_class(
&$loc:@optimizer//curiosity_value/bias*
dtype0*
valueB
 *fff?
�
optimizer/beta1_power_1
VariableV2*2
_class(
&$loc:@optimizer//curiosity_value/bias*
	container *
dtype0*
shape: *
shared_name 
�
optimizer/beta1_power_1/AssignAssignoptimizer/beta1_power_1%optimizer/beta1_power_1/initial_value*
T0*2
_class(
&$loc:@optimizer//curiosity_value/bias*
use_locking(*
validate_shape(
~
optimizer/beta1_power_1/readIdentityoptimizer/beta1_power_1*
T0*2
_class(
&$loc:@optimizer//curiosity_value/bias
�
%optimizer/beta2_power_1/initial_valueConst*2
_class(
&$loc:@optimizer//curiosity_value/bias*
dtype0*
valueB
 *w�?
�
optimizer/beta2_power_1
VariableV2*2
_class(
&$loc:@optimizer//curiosity_value/bias*
	container *
dtype0*
shape: *
shared_name 
�
optimizer/beta2_power_1/AssignAssignoptimizer/beta2_power_1%optimizer/beta2_power_1/initial_value*
T0*2
_class(
&$loc:@optimizer//curiosity_value/bias*
use_locking(*
validate_shape(
~
optimizer/beta2_power_1/readIdentityoptimizer/beta2_power_1*
T0*2
_class(
&$loc:@optimizer//curiosity_value/bias
�
Uoptimizer//policy/main_graph_0/hidden_0/kernel/Adam/Initializer/zeros/shape_as_tensorConst*6
_class,
*(loc:@policy/main_graph_0/hidden_0/kernel*
dtype0*
valueB"   �   
�
Koptimizer//policy/main_graph_0/hidden_0/kernel/Adam/Initializer/zeros/ConstConst*6
_class,
*(loc:@policy/main_graph_0/hidden_0/kernel*
dtype0*
valueB
 *    
�
Eoptimizer//policy/main_graph_0/hidden_0/kernel/Adam/Initializer/zerosFillUoptimizer//policy/main_graph_0/hidden_0/kernel/Adam/Initializer/zeros/shape_as_tensorKoptimizer//policy/main_graph_0/hidden_0/kernel/Adam/Initializer/zeros/Const*
T0*6
_class,
*(loc:@policy/main_graph_0/hidden_0/kernel*

index_type0
�
3optimizer//policy/main_graph_0/hidden_0/kernel/Adam
VariableV2*6
_class,
*(loc:@policy/main_graph_0/hidden_0/kernel*
	container *
dtype0*
shape:	�*
shared_name 
�
:optimizer//policy/main_graph_0/hidden_0/kernel/Adam/AssignAssign3optimizer//policy/main_graph_0/hidden_0/kernel/AdamEoptimizer//policy/main_graph_0/hidden_0/kernel/Adam/Initializer/zeros*
T0*6
_class,
*(loc:@policy/main_graph_0/hidden_0/kernel*
use_locking(*
validate_shape(
�
8optimizer//policy/main_graph_0/hidden_0/kernel/Adam/readIdentity3optimizer//policy/main_graph_0/hidden_0/kernel/Adam*
T0*6
_class,
*(loc:@policy/main_graph_0/hidden_0/kernel
�
Woptimizer//policy/main_graph_0/hidden_0/kernel/Adam_1/Initializer/zeros/shape_as_tensorConst*6
_class,
*(loc:@policy/main_graph_0/hidden_0/kernel*
dtype0*
valueB"   �   
�
Moptimizer//policy/main_graph_0/hidden_0/kernel/Adam_1/Initializer/zeros/ConstConst*6
_class,
*(loc:@policy/main_graph_0/hidden_0/kernel*
dtype0*
valueB
 *    
�
Goptimizer//policy/main_graph_0/hidden_0/kernel/Adam_1/Initializer/zerosFillWoptimizer//policy/main_graph_0/hidden_0/kernel/Adam_1/Initializer/zeros/shape_as_tensorMoptimizer//policy/main_graph_0/hidden_0/kernel/Adam_1/Initializer/zeros/Const*
T0*6
_class,
*(loc:@policy/main_graph_0/hidden_0/kernel*

index_type0
�
5optimizer//policy/main_graph_0/hidden_0/kernel/Adam_1
VariableV2*6
_class,
*(loc:@policy/main_graph_0/hidden_0/kernel*
	container *
dtype0*
shape:	�*
shared_name 
�
<optimizer//policy/main_graph_0/hidden_0/kernel/Adam_1/AssignAssign5optimizer//policy/main_graph_0/hidden_0/kernel/Adam_1Goptimizer//policy/main_graph_0/hidden_0/kernel/Adam_1/Initializer/zeros*
T0*6
_class,
*(loc:@policy/main_graph_0/hidden_0/kernel*
use_locking(*
validate_shape(
�
:optimizer//policy/main_graph_0/hidden_0/kernel/Adam_1/readIdentity5optimizer//policy/main_graph_0/hidden_0/kernel/Adam_1*
T0*6
_class,
*(loc:@policy/main_graph_0/hidden_0/kernel
�
Coptimizer//policy/main_graph_0/hidden_0/bias/Adam/Initializer/zerosConst*4
_class*
(&loc:@policy/main_graph_0/hidden_0/bias*
dtype0*
valueB�*    
�
1optimizer//policy/main_graph_0/hidden_0/bias/Adam
VariableV2*4
_class*
(&loc:@policy/main_graph_0/hidden_0/bias*
	container *
dtype0*
shape:�*
shared_name 
�
8optimizer//policy/main_graph_0/hidden_0/bias/Adam/AssignAssign1optimizer//policy/main_graph_0/hidden_0/bias/AdamCoptimizer//policy/main_graph_0/hidden_0/bias/Adam/Initializer/zeros*
T0*4
_class*
(&loc:@policy/main_graph_0/hidden_0/bias*
use_locking(*
validate_shape(
�
6optimizer//policy/main_graph_0/hidden_0/bias/Adam/readIdentity1optimizer//policy/main_graph_0/hidden_0/bias/Adam*
T0*4
_class*
(&loc:@policy/main_graph_0/hidden_0/bias
�
Eoptimizer//policy/main_graph_0/hidden_0/bias/Adam_1/Initializer/zerosConst*4
_class*
(&loc:@policy/main_graph_0/hidden_0/bias*
dtype0*
valueB�*    
�
3optimizer//policy/main_graph_0/hidden_0/bias/Adam_1
VariableV2*4
_class*
(&loc:@policy/main_graph_0/hidden_0/bias*
	container *
dtype0*
shape:�*
shared_name 
�
:optimizer//policy/main_graph_0/hidden_0/bias/Adam_1/AssignAssign3optimizer//policy/main_graph_0/hidden_0/bias/Adam_1Eoptimizer//policy/main_graph_0/hidden_0/bias/Adam_1/Initializer/zeros*
T0*4
_class*
(&loc:@policy/main_graph_0/hidden_0/bias*
use_locking(*
validate_shape(
�
8optimizer//policy/main_graph_0/hidden_0/bias/Adam_1/readIdentity3optimizer//policy/main_graph_0/hidden_0/bias/Adam_1*
T0*4
_class*
(&loc:@policy/main_graph_0/hidden_0/bias
�
Uoptimizer//policy/main_graph_0/hidden_1/kernel/Adam/Initializer/zeros/shape_as_tensorConst*6
_class,
*(loc:@policy/main_graph_0/hidden_1/kernel*
dtype0*
valueB"�   �   
�
Koptimizer//policy/main_graph_0/hidden_1/kernel/Adam/Initializer/zeros/ConstConst*6
_class,
*(loc:@policy/main_graph_0/hidden_1/kernel*
dtype0*
valueB
 *    
�
Eoptimizer//policy/main_graph_0/hidden_1/kernel/Adam/Initializer/zerosFillUoptimizer//policy/main_graph_0/hidden_1/kernel/Adam/Initializer/zeros/shape_as_tensorKoptimizer//policy/main_graph_0/hidden_1/kernel/Adam/Initializer/zeros/Const*
T0*6
_class,
*(loc:@policy/main_graph_0/hidden_1/kernel*

index_type0
�
3optimizer//policy/main_graph_0/hidden_1/kernel/Adam
VariableV2*6
_class,
*(loc:@policy/main_graph_0/hidden_1/kernel*
	container *
dtype0*
shape:
��*
shared_name 
�
:optimizer//policy/main_graph_0/hidden_1/kernel/Adam/AssignAssign3optimizer//policy/main_graph_0/hidden_1/kernel/AdamEoptimizer//policy/main_graph_0/hidden_1/kernel/Adam/Initializer/zeros*
T0*6
_class,
*(loc:@policy/main_graph_0/hidden_1/kernel*
use_locking(*
validate_shape(
�
8optimizer//policy/main_graph_0/hidden_1/kernel/Adam/readIdentity3optimizer//policy/main_graph_0/hidden_1/kernel/Adam*
T0*6
_class,
*(loc:@policy/main_graph_0/hidden_1/kernel
�
Woptimizer//policy/main_graph_0/hidden_1/kernel/Adam_1/Initializer/zeros/shape_as_tensorConst*6
_class,
*(loc:@policy/main_graph_0/hidden_1/kernel*
dtype0*
valueB"�   �   
�
Moptimizer//policy/main_graph_0/hidden_1/kernel/Adam_1/Initializer/zeros/ConstConst*6
_class,
*(loc:@policy/main_graph_0/hidden_1/kernel*
dtype0*
valueB
 *    
�
Goptimizer//policy/main_graph_0/hidden_1/kernel/Adam_1/Initializer/zerosFillWoptimizer//policy/main_graph_0/hidden_1/kernel/Adam_1/Initializer/zeros/shape_as_tensorMoptimizer//policy/main_graph_0/hidden_1/kernel/Adam_1/Initializer/zeros/Const*
T0*6
_class,
*(loc:@policy/main_graph_0/hidden_1/kernel*

index_type0
�
5optimizer//policy/main_graph_0/hidden_1/kernel/Adam_1
VariableV2*6
_class,
*(loc:@policy/main_graph_0/hidden_1/kernel*
	container *
dtype0*
shape:
��*
shared_name 
�
<optimizer//policy/main_graph_0/hidden_1/kernel/Adam_1/AssignAssign5optimizer//policy/main_graph_0/hidden_1/kernel/Adam_1Goptimizer//policy/main_graph_0/hidden_1/kernel/Adam_1/Initializer/zeros*
T0*6
_class,
*(loc:@policy/main_graph_0/hidden_1/kernel*
use_locking(*
validate_shape(
�
:optimizer//policy/main_graph_0/hidden_1/kernel/Adam_1/readIdentity5optimizer//policy/main_graph_0/hidden_1/kernel/Adam_1*
T0*6
_class,
*(loc:@policy/main_graph_0/hidden_1/kernel
�
Coptimizer//policy/main_graph_0/hidden_1/bias/Adam/Initializer/zerosConst*4
_class*
(&loc:@policy/main_graph_0/hidden_1/bias*
dtype0*
valueB�*    
�
1optimizer//policy/main_graph_0/hidden_1/bias/Adam
VariableV2*4
_class*
(&loc:@policy/main_graph_0/hidden_1/bias*
	container *
dtype0*
shape:�*
shared_name 
�
8optimizer//policy/main_graph_0/hidden_1/bias/Adam/AssignAssign1optimizer//policy/main_graph_0/hidden_1/bias/AdamCoptimizer//policy/main_graph_0/hidden_1/bias/Adam/Initializer/zeros*
T0*4
_class*
(&loc:@policy/main_graph_0/hidden_1/bias*
use_locking(*
validate_shape(
�
6optimizer//policy/main_graph_0/hidden_1/bias/Adam/readIdentity1optimizer//policy/main_graph_0/hidden_1/bias/Adam*
T0*4
_class*
(&loc:@policy/main_graph_0/hidden_1/bias
�
Eoptimizer//policy/main_graph_0/hidden_1/bias/Adam_1/Initializer/zerosConst*4
_class*
(&loc:@policy/main_graph_0/hidden_1/bias*
dtype0*
valueB�*    
�
3optimizer//policy/main_graph_0/hidden_1/bias/Adam_1
VariableV2*4
_class*
(&loc:@policy/main_graph_0/hidden_1/bias*
	container *
dtype0*
shape:�*
shared_name 
�
:optimizer//policy/main_graph_0/hidden_1/bias/Adam_1/AssignAssign3optimizer//policy/main_graph_0/hidden_1/bias/Adam_1Eoptimizer//policy/main_graph_0/hidden_1/bias/Adam_1/Initializer/zeros*
T0*4
_class*
(&loc:@policy/main_graph_0/hidden_1/bias*
use_locking(*
validate_shape(
�
8optimizer//policy/main_graph_0/hidden_1/bias/Adam_1/readIdentity3optimizer//policy/main_graph_0/hidden_1/bias/Adam_1*
T0*4
_class*
(&loc:@policy/main_graph_0/hidden_1/bias
�
5optimizer//policy/dense/kernel/Adam/Initializer/zerosConst*&
_class
loc:@policy/dense/kernel*
dtype0*
valueB	�*    
�
#optimizer//policy/dense/kernel/Adam
VariableV2*&
_class
loc:@policy/dense/kernel*
	container *
dtype0*
shape:	�*
shared_name 
�
*optimizer//policy/dense/kernel/Adam/AssignAssign#optimizer//policy/dense/kernel/Adam5optimizer//policy/dense/kernel/Adam/Initializer/zeros*
T0*&
_class
loc:@policy/dense/kernel*
use_locking(*
validate_shape(
�
(optimizer//policy/dense/kernel/Adam/readIdentity#optimizer//policy/dense/kernel/Adam*
T0*&
_class
loc:@policy/dense/kernel
�
7optimizer//policy/dense/kernel/Adam_1/Initializer/zerosConst*&
_class
loc:@policy/dense/kernel*
dtype0*
valueB	�*    
�
%optimizer//policy/dense/kernel/Adam_1
VariableV2*&
_class
loc:@policy/dense/kernel*
	container *
dtype0*
shape:	�*
shared_name 
�
,optimizer//policy/dense/kernel/Adam_1/AssignAssign%optimizer//policy/dense/kernel/Adam_17optimizer//policy/dense/kernel/Adam_1/Initializer/zeros*
T0*&
_class
loc:@policy/dense/kernel*
use_locking(*
validate_shape(
�
*optimizer//policy/dense/kernel/Adam_1/readIdentity%optimizer//policy/dense/kernel/Adam_1*
T0*&
_class
loc:@policy/dense/kernel
�
7optimizer//policy/dense_1/kernel/Adam/Initializer/zerosConst*(
_class
loc:@policy/dense_1/kernel*
dtype0*
valueB	�*    
�
%optimizer//policy/dense_1/kernel/Adam
VariableV2*(
_class
loc:@policy/dense_1/kernel*
	container *
dtype0*
shape:	�*
shared_name 
�
,optimizer//policy/dense_1/kernel/Adam/AssignAssign%optimizer//policy/dense_1/kernel/Adam7optimizer//policy/dense_1/kernel/Adam/Initializer/zeros*
T0*(
_class
loc:@policy/dense_1/kernel*
use_locking(*
validate_shape(
�
*optimizer//policy/dense_1/kernel/Adam/readIdentity%optimizer//policy/dense_1/kernel/Adam*
T0*(
_class
loc:@policy/dense_1/kernel
�
9optimizer//policy/dense_1/kernel/Adam_1/Initializer/zerosConst*(
_class
loc:@policy/dense_1/kernel*
dtype0*
valueB	�*    
�
'optimizer//policy/dense_1/kernel/Adam_1
VariableV2*(
_class
loc:@policy/dense_1/kernel*
	container *
dtype0*
shape:	�*
shared_name 
�
.optimizer//policy/dense_1/kernel/Adam_1/AssignAssign'optimizer//policy/dense_1/kernel/Adam_19optimizer//policy/dense_1/kernel/Adam_1/Initializer/zeros*
T0*(
_class
loc:@policy/dense_1/kernel*
use_locking(*
validate_shape(
�
,optimizer//policy/dense_1/kernel/Adam_1/readIdentity'optimizer//policy/dense_1/kernel/Adam_1*
T0*(
_class
loc:@policy/dense_1/kernel
�
Yoptimizer//optimizer//main_graph_0/hidden_0/kernel/Adam/Initializer/zeros/shape_as_tensorConst*:
_class0
.,loc:@optimizer//main_graph_0/hidden_0/kernel*
dtype0*
valueB"   �   
�
Ooptimizer//optimizer//main_graph_0/hidden_0/kernel/Adam/Initializer/zeros/ConstConst*:
_class0
.,loc:@optimizer//main_graph_0/hidden_0/kernel*
dtype0*
valueB
 *    
�
Ioptimizer//optimizer//main_graph_0/hidden_0/kernel/Adam/Initializer/zerosFillYoptimizer//optimizer//main_graph_0/hidden_0/kernel/Adam/Initializer/zeros/shape_as_tensorOoptimizer//optimizer//main_graph_0/hidden_0/kernel/Adam/Initializer/zeros/Const*
T0*:
_class0
.,loc:@optimizer//main_graph_0/hidden_0/kernel*

index_type0
�
7optimizer//optimizer//main_graph_0/hidden_0/kernel/Adam
VariableV2*:
_class0
.,loc:@optimizer//main_graph_0/hidden_0/kernel*
	container *
dtype0*
shape:	�*
shared_name 
�
>optimizer//optimizer//main_graph_0/hidden_0/kernel/Adam/AssignAssign7optimizer//optimizer//main_graph_0/hidden_0/kernel/AdamIoptimizer//optimizer//main_graph_0/hidden_0/kernel/Adam/Initializer/zeros*
T0*:
_class0
.,loc:@optimizer//main_graph_0/hidden_0/kernel*
use_locking(*
validate_shape(
�
<optimizer//optimizer//main_graph_0/hidden_0/kernel/Adam/readIdentity7optimizer//optimizer//main_graph_0/hidden_0/kernel/Adam*
T0*:
_class0
.,loc:@optimizer//main_graph_0/hidden_0/kernel
�
[optimizer//optimizer//main_graph_0/hidden_0/kernel/Adam_1/Initializer/zeros/shape_as_tensorConst*:
_class0
.,loc:@optimizer//main_graph_0/hidden_0/kernel*
dtype0*
valueB"   �   
�
Qoptimizer//optimizer//main_graph_0/hidden_0/kernel/Adam_1/Initializer/zeros/ConstConst*:
_class0
.,loc:@optimizer//main_graph_0/hidden_0/kernel*
dtype0*
valueB
 *    
�
Koptimizer//optimizer//main_graph_0/hidden_0/kernel/Adam_1/Initializer/zerosFill[optimizer//optimizer//main_graph_0/hidden_0/kernel/Adam_1/Initializer/zeros/shape_as_tensorQoptimizer//optimizer//main_graph_0/hidden_0/kernel/Adam_1/Initializer/zeros/Const*
T0*:
_class0
.,loc:@optimizer//main_graph_0/hidden_0/kernel*

index_type0
�
9optimizer//optimizer//main_graph_0/hidden_0/kernel/Adam_1
VariableV2*:
_class0
.,loc:@optimizer//main_graph_0/hidden_0/kernel*
	container *
dtype0*
shape:	�*
shared_name 
�
@optimizer//optimizer//main_graph_0/hidden_0/kernel/Adam_1/AssignAssign9optimizer//optimizer//main_graph_0/hidden_0/kernel/Adam_1Koptimizer//optimizer//main_graph_0/hidden_0/kernel/Adam_1/Initializer/zeros*
T0*:
_class0
.,loc:@optimizer//main_graph_0/hidden_0/kernel*
use_locking(*
validate_shape(
�
>optimizer//optimizer//main_graph_0/hidden_0/kernel/Adam_1/readIdentity9optimizer//optimizer//main_graph_0/hidden_0/kernel/Adam_1*
T0*:
_class0
.,loc:@optimizer//main_graph_0/hidden_0/kernel
�
Goptimizer//optimizer//main_graph_0/hidden_0/bias/Adam/Initializer/zerosConst*8
_class.
,*loc:@optimizer//main_graph_0/hidden_0/bias*
dtype0*
valueB�*    
�
5optimizer//optimizer//main_graph_0/hidden_0/bias/Adam
VariableV2*8
_class.
,*loc:@optimizer//main_graph_0/hidden_0/bias*
	container *
dtype0*
shape:�*
shared_name 
�
<optimizer//optimizer//main_graph_0/hidden_0/bias/Adam/AssignAssign5optimizer//optimizer//main_graph_0/hidden_0/bias/AdamGoptimizer//optimizer//main_graph_0/hidden_0/bias/Adam/Initializer/zeros*
T0*8
_class.
,*loc:@optimizer//main_graph_0/hidden_0/bias*
use_locking(*
validate_shape(
�
:optimizer//optimizer//main_graph_0/hidden_0/bias/Adam/readIdentity5optimizer//optimizer//main_graph_0/hidden_0/bias/Adam*
T0*8
_class.
,*loc:@optimizer//main_graph_0/hidden_0/bias
�
Ioptimizer//optimizer//main_graph_0/hidden_0/bias/Adam_1/Initializer/zerosConst*8
_class.
,*loc:@optimizer//main_graph_0/hidden_0/bias*
dtype0*
valueB�*    
�
7optimizer//optimizer//main_graph_0/hidden_0/bias/Adam_1
VariableV2*8
_class.
,*loc:@optimizer//main_graph_0/hidden_0/bias*
	container *
dtype0*
shape:�*
shared_name 
�
>optimizer//optimizer//main_graph_0/hidden_0/bias/Adam_1/AssignAssign7optimizer//optimizer//main_graph_0/hidden_0/bias/Adam_1Ioptimizer//optimizer//main_graph_0/hidden_0/bias/Adam_1/Initializer/zeros*
T0*8
_class.
,*loc:@optimizer//main_graph_0/hidden_0/bias*
use_locking(*
validate_shape(
�
<optimizer//optimizer//main_graph_0/hidden_0/bias/Adam_1/readIdentity7optimizer//optimizer//main_graph_0/hidden_0/bias/Adam_1*
T0*8
_class.
,*loc:@optimizer//main_graph_0/hidden_0/bias
�
Yoptimizer//optimizer//main_graph_0/hidden_1/kernel/Adam/Initializer/zeros/shape_as_tensorConst*:
_class0
.,loc:@optimizer//main_graph_0/hidden_1/kernel*
dtype0*
valueB"�   �   
�
Ooptimizer//optimizer//main_graph_0/hidden_1/kernel/Adam/Initializer/zeros/ConstConst*:
_class0
.,loc:@optimizer//main_graph_0/hidden_1/kernel*
dtype0*
valueB
 *    
�
Ioptimizer//optimizer//main_graph_0/hidden_1/kernel/Adam/Initializer/zerosFillYoptimizer//optimizer//main_graph_0/hidden_1/kernel/Adam/Initializer/zeros/shape_as_tensorOoptimizer//optimizer//main_graph_0/hidden_1/kernel/Adam/Initializer/zeros/Const*
T0*:
_class0
.,loc:@optimizer//main_graph_0/hidden_1/kernel*

index_type0
�
7optimizer//optimizer//main_graph_0/hidden_1/kernel/Adam
VariableV2*:
_class0
.,loc:@optimizer//main_graph_0/hidden_1/kernel*
	container *
dtype0*
shape:
��*
shared_name 
�
>optimizer//optimizer//main_graph_0/hidden_1/kernel/Adam/AssignAssign7optimizer//optimizer//main_graph_0/hidden_1/kernel/AdamIoptimizer//optimizer//main_graph_0/hidden_1/kernel/Adam/Initializer/zeros*
T0*:
_class0
.,loc:@optimizer//main_graph_0/hidden_1/kernel*
use_locking(*
validate_shape(
�
<optimizer//optimizer//main_graph_0/hidden_1/kernel/Adam/readIdentity7optimizer//optimizer//main_graph_0/hidden_1/kernel/Adam*
T0*:
_class0
.,loc:@optimizer//main_graph_0/hidden_1/kernel
�
[optimizer//optimizer//main_graph_0/hidden_1/kernel/Adam_1/Initializer/zeros/shape_as_tensorConst*:
_class0
.,loc:@optimizer//main_graph_0/hidden_1/kernel*
dtype0*
valueB"�   �   
�
Qoptimizer//optimizer//main_graph_0/hidden_1/kernel/Adam_1/Initializer/zeros/ConstConst*:
_class0
.,loc:@optimizer//main_graph_0/hidden_1/kernel*
dtype0*
valueB
 *    
�
Koptimizer//optimizer//main_graph_0/hidden_1/kernel/Adam_1/Initializer/zerosFill[optimizer//optimizer//main_graph_0/hidden_1/kernel/Adam_1/Initializer/zeros/shape_as_tensorQoptimizer//optimizer//main_graph_0/hidden_1/kernel/Adam_1/Initializer/zeros/Const*
T0*:
_class0
.,loc:@optimizer//main_graph_0/hidden_1/kernel*

index_type0
�
9optimizer//optimizer//main_graph_0/hidden_1/kernel/Adam_1
VariableV2*:
_class0
.,loc:@optimizer//main_graph_0/hidden_1/kernel*
	container *
dtype0*
shape:
��*
shared_name 
�
@optimizer//optimizer//main_graph_0/hidden_1/kernel/Adam_1/AssignAssign9optimizer//optimizer//main_graph_0/hidden_1/kernel/Adam_1Koptimizer//optimizer//main_graph_0/hidden_1/kernel/Adam_1/Initializer/zeros*
T0*:
_class0
.,loc:@optimizer//main_graph_0/hidden_1/kernel*
use_locking(*
validate_shape(
�
>optimizer//optimizer//main_graph_0/hidden_1/kernel/Adam_1/readIdentity9optimizer//optimizer//main_graph_0/hidden_1/kernel/Adam_1*
T0*:
_class0
.,loc:@optimizer//main_graph_0/hidden_1/kernel
�
Goptimizer//optimizer//main_graph_0/hidden_1/bias/Adam/Initializer/zerosConst*8
_class.
,*loc:@optimizer//main_graph_0/hidden_1/bias*
dtype0*
valueB�*    
�
5optimizer//optimizer//main_graph_0/hidden_1/bias/Adam
VariableV2*8
_class.
,*loc:@optimizer//main_graph_0/hidden_1/bias*
	container *
dtype0*
shape:�*
shared_name 
�
<optimizer//optimizer//main_graph_0/hidden_1/bias/Adam/AssignAssign5optimizer//optimizer//main_graph_0/hidden_1/bias/AdamGoptimizer//optimizer//main_graph_0/hidden_1/bias/Adam/Initializer/zeros*
T0*8
_class.
,*loc:@optimizer//main_graph_0/hidden_1/bias*
use_locking(*
validate_shape(
�
:optimizer//optimizer//main_graph_0/hidden_1/bias/Adam/readIdentity5optimizer//optimizer//main_graph_0/hidden_1/bias/Adam*
T0*8
_class.
,*loc:@optimizer//main_graph_0/hidden_1/bias
�
Ioptimizer//optimizer//main_graph_0/hidden_1/bias/Adam_1/Initializer/zerosConst*8
_class.
,*loc:@optimizer//main_graph_0/hidden_1/bias*
dtype0*
valueB�*    
�
7optimizer//optimizer//main_graph_0/hidden_1/bias/Adam_1
VariableV2*8
_class.
,*loc:@optimizer//main_graph_0/hidden_1/bias*
	container *
dtype0*
shape:�*
shared_name 
�
>optimizer//optimizer//main_graph_0/hidden_1/bias/Adam_1/AssignAssign7optimizer//optimizer//main_graph_0/hidden_1/bias/Adam_1Ioptimizer//optimizer//main_graph_0/hidden_1/bias/Adam_1/Initializer/zeros*
T0*8
_class.
,*loc:@optimizer//main_graph_0/hidden_1/bias*
use_locking(*
validate_shape(
�
<optimizer//optimizer//main_graph_0/hidden_1/bias/Adam_1/readIdentity7optimizer//optimizer//main_graph_0/hidden_1/bias/Adam_1*
T0*8
_class.
,*loc:@optimizer//main_graph_0/hidden_1/bias
�
Coptimizer//optimizer//extrinsic_value/kernel/Adam/Initializer/zerosConst*4
_class*
(&loc:@optimizer//extrinsic_value/kernel*
dtype0*
valueB	�*    
�
1optimizer//optimizer//extrinsic_value/kernel/Adam
VariableV2*4
_class*
(&loc:@optimizer//extrinsic_value/kernel*
	container *
dtype0*
shape:	�*
shared_name 
�
8optimizer//optimizer//extrinsic_value/kernel/Adam/AssignAssign1optimizer//optimizer//extrinsic_value/kernel/AdamCoptimizer//optimizer//extrinsic_value/kernel/Adam/Initializer/zeros*
T0*4
_class*
(&loc:@optimizer//extrinsic_value/kernel*
use_locking(*
validate_shape(
�
6optimizer//optimizer//extrinsic_value/kernel/Adam/readIdentity1optimizer//optimizer//extrinsic_value/kernel/Adam*
T0*4
_class*
(&loc:@optimizer//extrinsic_value/kernel
�
Eoptimizer//optimizer//extrinsic_value/kernel/Adam_1/Initializer/zerosConst*4
_class*
(&loc:@optimizer//extrinsic_value/kernel*
dtype0*
valueB	�*    
�
3optimizer//optimizer//extrinsic_value/kernel/Adam_1
VariableV2*4
_class*
(&loc:@optimizer//extrinsic_value/kernel*
	container *
dtype0*
shape:	�*
shared_name 
�
:optimizer//optimizer//extrinsic_value/kernel/Adam_1/AssignAssign3optimizer//optimizer//extrinsic_value/kernel/Adam_1Eoptimizer//optimizer//extrinsic_value/kernel/Adam_1/Initializer/zeros*
T0*4
_class*
(&loc:@optimizer//extrinsic_value/kernel*
use_locking(*
validate_shape(
�
8optimizer//optimizer//extrinsic_value/kernel/Adam_1/readIdentity3optimizer//optimizer//extrinsic_value/kernel/Adam_1*
T0*4
_class*
(&loc:@optimizer//extrinsic_value/kernel
�
Aoptimizer//optimizer//extrinsic_value/bias/Adam/Initializer/zerosConst*2
_class(
&$loc:@optimizer//extrinsic_value/bias*
dtype0*
valueB*    
�
/optimizer//optimizer//extrinsic_value/bias/Adam
VariableV2*2
_class(
&$loc:@optimizer//extrinsic_value/bias*
	container *
dtype0*
shape:*
shared_name 
�
6optimizer//optimizer//extrinsic_value/bias/Adam/AssignAssign/optimizer//optimizer//extrinsic_value/bias/AdamAoptimizer//optimizer//extrinsic_value/bias/Adam/Initializer/zeros*
T0*2
_class(
&$loc:@optimizer//extrinsic_value/bias*
use_locking(*
validate_shape(
�
4optimizer//optimizer//extrinsic_value/bias/Adam/readIdentity/optimizer//optimizer//extrinsic_value/bias/Adam*
T0*2
_class(
&$loc:@optimizer//extrinsic_value/bias
�
Coptimizer//optimizer//extrinsic_value/bias/Adam_1/Initializer/zerosConst*2
_class(
&$loc:@optimizer//extrinsic_value/bias*
dtype0*
valueB*    
�
1optimizer//optimizer//extrinsic_value/bias/Adam_1
VariableV2*2
_class(
&$loc:@optimizer//extrinsic_value/bias*
	container *
dtype0*
shape:*
shared_name 
�
8optimizer//optimizer//extrinsic_value/bias/Adam_1/AssignAssign1optimizer//optimizer//extrinsic_value/bias/Adam_1Coptimizer//optimizer//extrinsic_value/bias/Adam_1/Initializer/zeros*
T0*2
_class(
&$loc:@optimizer//extrinsic_value/bias*
use_locking(*
validate_shape(
�
6optimizer//optimizer//extrinsic_value/bias/Adam_1/readIdentity1optimizer//optimizer//extrinsic_value/bias/Adam_1*
T0*2
_class(
&$loc:@optimizer//extrinsic_value/bias
�
Coptimizer//optimizer//curiosity_value/kernel/Adam/Initializer/zerosConst*4
_class*
(&loc:@optimizer//curiosity_value/kernel*
dtype0*
valueB	�*    
�
1optimizer//optimizer//curiosity_value/kernel/Adam
VariableV2*4
_class*
(&loc:@optimizer//curiosity_value/kernel*
	container *
dtype0*
shape:	�*
shared_name 
�
8optimizer//optimizer//curiosity_value/kernel/Adam/AssignAssign1optimizer//optimizer//curiosity_value/kernel/AdamCoptimizer//optimizer//curiosity_value/kernel/Adam/Initializer/zeros*
T0*4
_class*
(&loc:@optimizer//curiosity_value/kernel*
use_locking(*
validate_shape(
�
6optimizer//optimizer//curiosity_value/kernel/Adam/readIdentity1optimizer//optimizer//curiosity_value/kernel/Adam*
T0*4
_class*
(&loc:@optimizer//curiosity_value/kernel
�
Eoptimizer//optimizer//curiosity_value/kernel/Adam_1/Initializer/zerosConst*4
_class*
(&loc:@optimizer//curiosity_value/kernel*
dtype0*
valueB	�*    
�
3optimizer//optimizer//curiosity_value/kernel/Adam_1
VariableV2*4
_class*
(&loc:@optimizer//curiosity_value/kernel*
	container *
dtype0*
shape:	�*
shared_name 
�
:optimizer//optimizer//curiosity_value/kernel/Adam_1/AssignAssign3optimizer//optimizer//curiosity_value/kernel/Adam_1Eoptimizer//optimizer//curiosity_value/kernel/Adam_1/Initializer/zeros*
T0*4
_class*
(&loc:@optimizer//curiosity_value/kernel*
use_locking(*
validate_shape(
�
8optimizer//optimizer//curiosity_value/kernel/Adam_1/readIdentity3optimizer//optimizer//curiosity_value/kernel/Adam_1*
T0*4
_class*
(&loc:@optimizer//curiosity_value/kernel
�
Aoptimizer//optimizer//curiosity_value/bias/Adam/Initializer/zerosConst*2
_class(
&$loc:@optimizer//curiosity_value/bias*
dtype0*
valueB*    
�
/optimizer//optimizer//curiosity_value/bias/Adam
VariableV2*2
_class(
&$loc:@optimizer//curiosity_value/bias*
	container *
dtype0*
shape:*
shared_name 
�
6optimizer//optimizer//curiosity_value/bias/Adam/AssignAssign/optimizer//optimizer//curiosity_value/bias/AdamAoptimizer//optimizer//curiosity_value/bias/Adam/Initializer/zeros*
T0*2
_class(
&$loc:@optimizer//curiosity_value/bias*
use_locking(*
validate_shape(
�
4optimizer//optimizer//curiosity_value/bias/Adam/readIdentity/optimizer//optimizer//curiosity_value/bias/Adam*
T0*2
_class(
&$loc:@optimizer//curiosity_value/bias
�
Coptimizer//optimizer//curiosity_value/bias/Adam_1/Initializer/zerosConst*2
_class(
&$loc:@optimizer//curiosity_value/bias*
dtype0*
valueB*    
�
1optimizer//optimizer//curiosity_value/bias/Adam_1
VariableV2*2
_class(
&$loc:@optimizer//curiosity_value/bias*
	container *
dtype0*
shape:*
shared_name 
�
8optimizer//optimizer//curiosity_value/bias/Adam_1/AssignAssign1optimizer//optimizer//curiosity_value/bias/Adam_1Coptimizer//optimizer//curiosity_value/bias/Adam_1/Initializer/zeros*
T0*2
_class(
&$loc:@optimizer//curiosity_value/bias*
use_locking(*
validate_shape(
�
6optimizer//optimizer//curiosity_value/bias/Adam_1/readIdentity1optimizer//optimizer//curiosity_value/bias/Adam_1*
T0*2
_class(
&$loc:@optimizer//curiosity_value/bias
C
optimizer/Adam_1/beta1Const*
dtype0*
valueB
 *fff?
C
optimizer/Adam_1/beta2Const*
dtype0*
valueB
 *w�?
E
optimizer/Adam_1/epsilonConst*
dtype0*
valueB
 *w�+2
�
Eoptimizer/Adam_1/update_policy/main_graph_0/hidden_0/kernel/ApplyAdam	ApplyAdam#policy/main_graph_0/hidden_0/kernel3optimizer//policy/main_graph_0/hidden_0/kernel/Adam5optimizer//policy/main_graph_0/hidden_0/kernel/Adam_1optimizer/beta1_power_1/readoptimizer/beta2_power_1/readoptimizer/Variable/readoptimizer/Adam_1/beta1optimizer/Adam_1/beta2optimizer/Adam_1/epsilonYoptimizer/gradients_2/policy/main_graph_0/hidden_0/MatMul_grad/tuple/control_dependency_1*
T0*6
_class,
*(loc:@policy/main_graph_0/hidden_0/kernel*
use_locking( *
use_nesterov( 
�
Coptimizer/Adam_1/update_policy/main_graph_0/hidden_0/bias/ApplyAdam	ApplyAdam!policy/main_graph_0/hidden_0/bias1optimizer//policy/main_graph_0/hidden_0/bias/Adam3optimizer//policy/main_graph_0/hidden_0/bias/Adam_1optimizer/beta1_power_1/readoptimizer/beta2_power_1/readoptimizer/Variable/readoptimizer/Adam_1/beta1optimizer/Adam_1/beta2optimizer/Adam_1/epsilonZoptimizer/gradients_2/policy/main_graph_0/hidden_0/BiasAdd_grad/tuple/control_dependency_1*
T0*4
_class*
(&loc:@policy/main_graph_0/hidden_0/bias*
use_locking( *
use_nesterov( 
�
Eoptimizer/Adam_1/update_policy/main_graph_0/hidden_1/kernel/ApplyAdam	ApplyAdam#policy/main_graph_0/hidden_1/kernel3optimizer//policy/main_graph_0/hidden_1/kernel/Adam5optimizer//policy/main_graph_0/hidden_1/kernel/Adam_1optimizer/beta1_power_1/readoptimizer/beta2_power_1/readoptimizer/Variable/readoptimizer/Adam_1/beta1optimizer/Adam_1/beta2optimizer/Adam_1/epsilonYoptimizer/gradients_2/policy/main_graph_0/hidden_1/MatMul_grad/tuple/control_dependency_1*
T0*6
_class,
*(loc:@policy/main_graph_0/hidden_1/kernel*
use_locking( *
use_nesterov( 
�
Coptimizer/Adam_1/update_policy/main_graph_0/hidden_1/bias/ApplyAdam	ApplyAdam!policy/main_graph_0/hidden_1/bias1optimizer//policy/main_graph_0/hidden_1/bias/Adam3optimizer//policy/main_graph_0/hidden_1/bias/Adam_1optimizer/beta1_power_1/readoptimizer/beta2_power_1/readoptimizer/Variable/readoptimizer/Adam_1/beta1optimizer/Adam_1/beta2optimizer/Adam_1/epsilonZoptimizer/gradients_2/policy/main_graph_0/hidden_1/BiasAdd_grad/tuple/control_dependency_1*
T0*4
_class*
(&loc:@policy/main_graph_0/hidden_1/bias*
use_locking( *
use_nesterov( 
�
5optimizer/Adam_1/update_policy/dense/kernel/ApplyAdam	ApplyAdampolicy/dense/kernel#optimizer//policy/dense/kernel/Adam%optimizer//policy/dense/kernel/Adam_1optimizer/beta1_power_1/readoptimizer/beta2_power_1/readoptimizer/Variable/readoptimizer/Adam_1/beta1optimizer/Adam_1/beta2optimizer/Adam_1/epsilonKoptimizer/gradients_2/policy_1/dense/MatMul_grad/tuple/control_dependency_1*
T0*&
_class
loc:@policy/dense/kernel*
use_locking( *
use_nesterov( 
�
7optimizer/Adam_1/update_policy/dense_1/kernel/ApplyAdam	ApplyAdampolicy/dense_1/kernel%optimizer//policy/dense_1/kernel/Adam'optimizer//policy/dense_1/kernel/Adam_1optimizer/beta1_power_1/readoptimizer/beta2_power_1/readoptimizer/Variable/readoptimizer/Adam_1/beta1optimizer/Adam_1/beta2optimizer/Adam_1/epsilonMoptimizer/gradients_2/policy_1/dense_1/MatMul_grad/tuple/control_dependency_1*
T0*(
_class
loc:@policy/dense_1/kernel*
use_locking( *
use_nesterov( 
�
Ioptimizer/Adam_1/update_optimizer//main_graph_0/hidden_0/kernel/ApplyAdam	ApplyAdam'optimizer//main_graph_0/hidden_0/kernel7optimizer//optimizer//main_graph_0/hidden_0/kernel/Adam9optimizer//optimizer//main_graph_0/hidden_0/kernel/Adam_1optimizer/beta1_power_1/readoptimizer/beta2_power_1/readoptimizer/Variable/readoptimizer/Adam_1/beta1optimizer/Adam_1/beta2optimizer/Adam_1/epsilon\optimizer/gradients_2/optimizer/main_graph_0/hidden_0/MatMul_grad/tuple/control_dependency_1*
T0*:
_class0
.,loc:@optimizer//main_graph_0/hidden_0/kernel*
use_locking( *
use_nesterov( 
�
Goptimizer/Adam_1/update_optimizer//main_graph_0/hidden_0/bias/ApplyAdam	ApplyAdam%optimizer//main_graph_0/hidden_0/bias5optimizer//optimizer//main_graph_0/hidden_0/bias/Adam7optimizer//optimizer//main_graph_0/hidden_0/bias/Adam_1optimizer/beta1_power_1/readoptimizer/beta2_power_1/readoptimizer/Variable/readoptimizer/Adam_1/beta1optimizer/Adam_1/beta2optimizer/Adam_1/epsilon]optimizer/gradients_2/optimizer/main_graph_0/hidden_0/BiasAdd_grad/tuple/control_dependency_1*
T0*8
_class.
,*loc:@optimizer//main_graph_0/hidden_0/bias*
use_locking( *
use_nesterov( 
�
Ioptimizer/Adam_1/update_optimizer//main_graph_0/hidden_1/kernel/ApplyAdam	ApplyAdam'optimizer//main_graph_0/hidden_1/kernel7optimizer//optimizer//main_graph_0/hidden_1/kernel/Adam9optimizer//optimizer//main_graph_0/hidden_1/kernel/Adam_1optimizer/beta1_power_1/readoptimizer/beta2_power_1/readoptimizer/Variable/readoptimizer/Adam_1/beta1optimizer/Adam_1/beta2optimizer/Adam_1/epsilon\optimizer/gradients_2/optimizer/main_graph_0/hidden_1/MatMul_grad/tuple/control_dependency_1*
T0*:
_class0
.,loc:@optimizer//main_graph_0/hidden_1/kernel*
use_locking( *
use_nesterov( 
�
Goptimizer/Adam_1/update_optimizer//main_graph_0/hidden_1/bias/ApplyAdam	ApplyAdam%optimizer//main_graph_0/hidden_1/bias5optimizer//optimizer//main_graph_0/hidden_1/bias/Adam7optimizer//optimizer//main_graph_0/hidden_1/bias/Adam_1optimizer/beta1_power_1/readoptimizer/beta2_power_1/readoptimizer/Variable/readoptimizer/Adam_1/beta1optimizer/Adam_1/beta2optimizer/Adam_1/epsilon]optimizer/gradients_2/optimizer/main_graph_0/hidden_1/BiasAdd_grad/tuple/control_dependency_1*
T0*8
_class.
,*loc:@optimizer//main_graph_0/hidden_1/bias*
use_locking( *
use_nesterov( 
�
Coptimizer/Adam_1/update_optimizer//extrinsic_value/kernel/ApplyAdam	ApplyAdam!optimizer//extrinsic_value/kernel1optimizer//optimizer//extrinsic_value/kernel/Adam3optimizer//optimizer//extrinsic_value/kernel/Adam_1optimizer/beta1_power_1/readoptimizer/beta2_power_1/readoptimizer/Variable/readoptimizer/Adam_1/beta1optimizer/Adam_1/beta2optimizer/Adam_1/epsilonVoptimizer/gradients_2/optimizer/extrinsic_value/MatMul_grad/tuple/control_dependency_1*
T0*4
_class*
(&loc:@optimizer//extrinsic_value/kernel*
use_locking( *
use_nesterov( 
�
Aoptimizer/Adam_1/update_optimizer//extrinsic_value/bias/ApplyAdam	ApplyAdamoptimizer//extrinsic_value/bias/optimizer//optimizer//extrinsic_value/bias/Adam1optimizer//optimizer//extrinsic_value/bias/Adam_1optimizer/beta1_power_1/readoptimizer/beta2_power_1/readoptimizer/Variable/readoptimizer/Adam_1/beta1optimizer/Adam_1/beta2optimizer/Adam_1/epsilonWoptimizer/gradients_2/optimizer/extrinsic_value/BiasAdd_grad/tuple/control_dependency_1*
T0*2
_class(
&$loc:@optimizer//extrinsic_value/bias*
use_locking( *
use_nesterov( 
�
Coptimizer/Adam_1/update_optimizer//curiosity_value/kernel/ApplyAdam	ApplyAdam!optimizer//curiosity_value/kernel1optimizer//optimizer//curiosity_value/kernel/Adam3optimizer//optimizer//curiosity_value/kernel/Adam_1optimizer/beta1_power_1/readoptimizer/beta2_power_1/readoptimizer/Variable/readoptimizer/Adam_1/beta1optimizer/Adam_1/beta2optimizer/Adam_1/epsilonVoptimizer/gradients_2/optimizer/curiosity_value/MatMul_grad/tuple/control_dependency_1*
T0*4
_class*
(&loc:@optimizer//curiosity_value/kernel*
use_locking( *
use_nesterov( 
�
Aoptimizer/Adam_1/update_optimizer//curiosity_value/bias/ApplyAdam	ApplyAdamoptimizer//curiosity_value/bias/optimizer//optimizer//curiosity_value/bias/Adam1optimizer//optimizer//curiosity_value/bias/Adam_1optimizer/beta1_power_1/readoptimizer/beta2_power_1/readoptimizer/Variable/readoptimizer/Adam_1/beta1optimizer/Adam_1/beta2optimizer/Adam_1/epsilonWoptimizer/gradients_2/optimizer/curiosity_value/BiasAdd_grad/tuple/control_dependency_1*
T0*2
_class(
&$loc:@optimizer//curiosity_value/bias*
use_locking( *
use_nesterov( 
�
optimizer/Adam_1/mulMuloptimizer/beta1_power_1/readoptimizer/Adam_1/beta1B^optimizer/Adam_1/update_optimizer//curiosity_value/bias/ApplyAdamD^optimizer/Adam_1/update_optimizer//curiosity_value/kernel/ApplyAdamB^optimizer/Adam_1/update_optimizer//extrinsic_value/bias/ApplyAdamD^optimizer/Adam_1/update_optimizer//extrinsic_value/kernel/ApplyAdamH^optimizer/Adam_1/update_optimizer//main_graph_0/hidden_0/bias/ApplyAdamJ^optimizer/Adam_1/update_optimizer//main_graph_0/hidden_0/kernel/ApplyAdamH^optimizer/Adam_1/update_optimizer//main_graph_0/hidden_1/bias/ApplyAdamJ^optimizer/Adam_1/update_optimizer//main_graph_0/hidden_1/kernel/ApplyAdam6^optimizer/Adam_1/update_policy/dense/kernel/ApplyAdam8^optimizer/Adam_1/update_policy/dense_1/kernel/ApplyAdamD^optimizer/Adam_1/update_policy/main_graph_0/hidden_0/bias/ApplyAdamF^optimizer/Adam_1/update_policy/main_graph_0/hidden_0/kernel/ApplyAdamD^optimizer/Adam_1/update_policy/main_graph_0/hidden_1/bias/ApplyAdamF^optimizer/Adam_1/update_policy/main_graph_0/hidden_1/kernel/ApplyAdam*
T0*2
_class(
&$loc:@optimizer//curiosity_value/bias
�
optimizer/Adam_1/AssignAssignoptimizer/beta1_power_1optimizer/Adam_1/mul*
T0*2
_class(
&$loc:@optimizer//curiosity_value/bias*
use_locking( *
validate_shape(
�
optimizer/Adam_1/mul_1Muloptimizer/beta2_power_1/readoptimizer/Adam_1/beta2B^optimizer/Adam_1/update_optimizer//curiosity_value/bias/ApplyAdamD^optimizer/Adam_1/update_optimizer//curiosity_value/kernel/ApplyAdamB^optimizer/Adam_1/update_optimizer//extrinsic_value/bias/ApplyAdamD^optimizer/Adam_1/update_optimizer//extrinsic_value/kernel/ApplyAdamH^optimizer/Adam_1/update_optimizer//main_graph_0/hidden_0/bias/ApplyAdamJ^optimizer/Adam_1/update_optimizer//main_graph_0/hidden_0/kernel/ApplyAdamH^optimizer/Adam_1/update_optimizer//main_graph_0/hidden_1/bias/ApplyAdamJ^optimizer/Adam_1/update_optimizer//main_graph_0/hidden_1/kernel/ApplyAdam6^optimizer/Adam_1/update_policy/dense/kernel/ApplyAdam8^optimizer/Adam_1/update_policy/dense_1/kernel/ApplyAdamD^optimizer/Adam_1/update_policy/main_graph_0/hidden_0/bias/ApplyAdamF^optimizer/Adam_1/update_policy/main_graph_0/hidden_0/kernel/ApplyAdamD^optimizer/Adam_1/update_policy/main_graph_0/hidden_1/bias/ApplyAdamF^optimizer/Adam_1/update_policy/main_graph_0/hidden_1/kernel/ApplyAdam*
T0*2
_class(
&$loc:@optimizer//curiosity_value/bias
�
optimizer/Adam_1/Assign_1Assignoptimizer/beta2_power_1optimizer/Adam_1/mul_1*
T0*2
_class(
&$loc:@optimizer//curiosity_value/bias*
use_locking( *
validate_shape(
�
optimizer/Adam_1NoOp^optimizer/Adam_1/Assign^optimizer/Adam_1/Assign_1B^optimizer/Adam_1/update_optimizer//curiosity_value/bias/ApplyAdamD^optimizer/Adam_1/update_optimizer//curiosity_value/kernel/ApplyAdamB^optimizer/Adam_1/update_optimizer//extrinsic_value/bias/ApplyAdamD^optimizer/Adam_1/update_optimizer//extrinsic_value/kernel/ApplyAdamH^optimizer/Adam_1/update_optimizer//main_graph_0/hidden_0/bias/ApplyAdamJ^optimizer/Adam_1/update_optimizer//main_graph_0/hidden_0/kernel/ApplyAdamH^optimizer/Adam_1/update_optimizer//main_graph_0/hidden_1/bias/ApplyAdamJ^optimizer/Adam_1/update_optimizer//main_graph_0/hidden_1/kernel/ApplyAdam6^optimizer/Adam_1/update_policy/dense/kernel/ApplyAdam8^optimizer/Adam_1/update_policy/dense_1/kernel/ApplyAdamD^optimizer/Adam_1/update_policy/main_graph_0/hidden_0/bias/ApplyAdamF^optimizer/Adam_1/update_policy/main_graph_0/hidden_0/kernel/ApplyAdamD^optimizer/Adam_1/update_policy/main_graph_0/hidden_1/bias/ApplyAdamF^optimizer/Adam_1/update_policy/main_graph_0/hidden_1/kernel/ApplyAdam
C
save_1/filename/inputConst*
dtype0*
valueB Bmodel
Z
save_1/filenamePlaceholderWithDefaultsave_1/filename/input*
dtype0*
shape: 
Q
save_1/ConstPlaceholderWithDefaultsave_1/filename*
dtype0*
shape: 
�!
save_1/SaveV2/tensor_namesConst*
dtype0*� 
value� B� cBaction_output_shapeBglobal_stepBis_continuous_controlBmemory_sizeBoptimizer//curiosity_value/biasB!optimizer//curiosity_value/kernelB5optimizer//curiosity_vector_obs_encoder/hidden_0/biasB7optimizer//curiosity_vector_obs_encoder/hidden_0/kernelB5optimizer//curiosity_vector_obs_encoder/hidden_1/biasB7optimizer//curiosity_vector_obs_encoder/hidden_1/kernelBoptimizer//dense/biasBoptimizer//dense/kernelBoptimizer//dense_1/biasBoptimizer//dense_1/kernelBoptimizer//dense_2/biasBoptimizer//dense_2/kernelBoptimizer//dense_3/biasBoptimizer//dense_3/kernelBoptimizer//dense_4/biasBoptimizer//dense_4/kernelBoptimizer//extrinsic_value/biasB!optimizer//extrinsic_value/kernelB%optimizer//main_graph_0/hidden_0/biasB'optimizer//main_graph_0/hidden_0/kernelB%optimizer//main_graph_0/hidden_1/biasB'optimizer//main_graph_0/hidden_1/kernelB/optimizer//optimizer//curiosity_value/bias/AdamB1optimizer//optimizer//curiosity_value/bias/Adam_1B1optimizer//optimizer//curiosity_value/kernel/AdamB3optimizer//optimizer//curiosity_value/kernel/Adam_1BEoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/bias/AdamBGoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/bias/Adam_1BGoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/kernel/AdamBIoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/kernel/Adam_1BEoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/bias/AdamBGoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/bias/Adam_1BGoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/kernel/AdamBIoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/kernel/Adam_1B%optimizer//optimizer//dense/bias/AdamB'optimizer//optimizer//dense/bias/Adam_1B'optimizer//optimizer//dense/kernel/AdamB)optimizer//optimizer//dense/kernel/Adam_1B'optimizer//optimizer//dense_1/bias/AdamB)optimizer//optimizer//dense_1/bias/Adam_1B)optimizer//optimizer//dense_1/kernel/AdamB+optimizer//optimizer//dense_1/kernel/Adam_1B'optimizer//optimizer//dense_2/bias/AdamB)optimizer//optimizer//dense_2/bias/Adam_1B)optimizer//optimizer//dense_2/kernel/AdamB+optimizer//optimizer//dense_2/kernel/Adam_1B'optimizer//optimizer//dense_3/bias/AdamB)optimizer//optimizer//dense_3/bias/Adam_1B)optimizer//optimizer//dense_3/kernel/AdamB+optimizer//optimizer//dense_3/kernel/Adam_1B'optimizer//optimizer//dense_4/bias/AdamB)optimizer//optimizer//dense_4/bias/Adam_1B)optimizer//optimizer//dense_4/kernel/AdamB+optimizer//optimizer//dense_4/kernel/Adam_1B/optimizer//optimizer//extrinsic_value/bias/AdamB1optimizer//optimizer//extrinsic_value/bias/Adam_1B1optimizer//optimizer//extrinsic_value/kernel/AdamB3optimizer//optimizer//extrinsic_value/kernel/Adam_1B5optimizer//optimizer//main_graph_0/hidden_0/bias/AdamB7optimizer//optimizer//main_graph_0/hidden_0/bias/Adam_1B7optimizer//optimizer//main_graph_0/hidden_0/kernel/AdamB9optimizer//optimizer//main_graph_0/hidden_0/kernel/Adam_1B5optimizer//optimizer//main_graph_0/hidden_1/bias/AdamB7optimizer//optimizer//main_graph_0/hidden_1/bias/Adam_1B7optimizer//optimizer//main_graph_0/hidden_1/kernel/AdamB9optimizer//optimizer//main_graph_0/hidden_1/kernel/Adam_1B#optimizer//policy/dense/kernel/AdamB%optimizer//policy/dense/kernel/Adam_1B%optimizer//policy/dense_1/kernel/AdamB'optimizer//policy/dense_1/kernel/Adam_1B1optimizer//policy/main_graph_0/hidden_0/bias/AdamB3optimizer//policy/main_graph_0/hidden_0/bias/Adam_1B3optimizer//policy/main_graph_0/hidden_0/kernel/AdamB5optimizer//policy/main_graph_0/hidden_0/kernel/Adam_1B1optimizer//policy/main_graph_0/hidden_1/bias/AdamB3optimizer//policy/main_graph_0/hidden_1/bias/Adam_1B3optimizer//policy/main_graph_0/hidden_1/kernel/AdamB5optimizer//policy/main_graph_0/hidden_1/kernel/Adam_1Boptimizer/VariableBoptimizer/Variable_1Boptimizer/Variable_2Boptimizer/beta1_powerBoptimizer/beta1_power_1Boptimizer/beta2_powerBoptimizer/beta2_power_1Bpolicy/dense/kernelBpolicy/dense_1/kernelB!policy/main_graph_0/hidden_0/biasB#policy/main_graph_0/hidden_0/kernelB!policy/main_graph_0/hidden_1/biasB#policy/main_graph_0/hidden_1/kernelBtrainer_major_versionBtrainer_minor_versionBtrainer_patch_versionBversion_number
�
save_1/SaveV2/shape_and_slicesConst*
dtype0*�
value�B�cB B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B 
�"
save_1/SaveV2SaveV2save_1/Constsave_1/SaveV2/tensor_namessave_1/SaveV2/shape_and_slicesaction_output_shapeglobal_stepis_continuous_controlmemory_sizeoptimizer//curiosity_value/bias!optimizer//curiosity_value/kernel5optimizer//curiosity_vector_obs_encoder/hidden_0/bias7optimizer//curiosity_vector_obs_encoder/hidden_0/kernel5optimizer//curiosity_vector_obs_encoder/hidden_1/bias7optimizer//curiosity_vector_obs_encoder/hidden_1/kerneloptimizer//dense/biasoptimizer//dense/kerneloptimizer//dense_1/biasoptimizer//dense_1/kerneloptimizer//dense_2/biasoptimizer//dense_2/kerneloptimizer//dense_3/biasoptimizer//dense_3/kerneloptimizer//dense_4/biasoptimizer//dense_4/kerneloptimizer//extrinsic_value/bias!optimizer//extrinsic_value/kernel%optimizer//main_graph_0/hidden_0/bias'optimizer//main_graph_0/hidden_0/kernel%optimizer//main_graph_0/hidden_1/bias'optimizer//main_graph_0/hidden_1/kernel/optimizer//optimizer//curiosity_value/bias/Adam1optimizer//optimizer//curiosity_value/bias/Adam_11optimizer//optimizer//curiosity_value/kernel/Adam3optimizer//optimizer//curiosity_value/kernel/Adam_1Eoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/bias/AdamGoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/bias/Adam_1Goptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/kernel/AdamIoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/kernel/Adam_1Eoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/bias/AdamGoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/bias/Adam_1Goptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/kernel/AdamIoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/kernel/Adam_1%optimizer//optimizer//dense/bias/Adam'optimizer//optimizer//dense/bias/Adam_1'optimizer//optimizer//dense/kernel/Adam)optimizer//optimizer//dense/kernel/Adam_1'optimizer//optimizer//dense_1/bias/Adam)optimizer//optimizer//dense_1/bias/Adam_1)optimizer//optimizer//dense_1/kernel/Adam+optimizer//optimizer//dense_1/kernel/Adam_1'optimizer//optimizer//dense_2/bias/Adam)optimizer//optimizer//dense_2/bias/Adam_1)optimizer//optimizer//dense_2/kernel/Adam+optimizer//optimizer//dense_2/kernel/Adam_1'optimizer//optimizer//dense_3/bias/Adam)optimizer//optimizer//dense_3/bias/Adam_1)optimizer//optimizer//dense_3/kernel/Adam+optimizer//optimizer//dense_3/kernel/Adam_1'optimizer//optimizer//dense_4/bias/Adam)optimizer//optimizer//dense_4/bias/Adam_1)optimizer//optimizer//dense_4/kernel/Adam+optimizer//optimizer//dense_4/kernel/Adam_1/optimizer//optimizer//extrinsic_value/bias/Adam1optimizer//optimizer//extrinsic_value/bias/Adam_11optimizer//optimizer//extrinsic_value/kernel/Adam3optimizer//optimizer//extrinsic_value/kernel/Adam_15optimizer//optimizer//main_graph_0/hidden_0/bias/Adam7optimizer//optimizer//main_graph_0/hidden_0/bias/Adam_17optimizer//optimizer//main_graph_0/hidden_0/kernel/Adam9optimizer//optimizer//main_graph_0/hidden_0/kernel/Adam_15optimizer//optimizer//main_graph_0/hidden_1/bias/Adam7optimizer//optimizer//main_graph_0/hidden_1/bias/Adam_17optimizer//optimizer//main_graph_0/hidden_1/kernel/Adam9optimizer//optimizer//main_graph_0/hidden_1/kernel/Adam_1#optimizer//policy/dense/kernel/Adam%optimizer//policy/dense/kernel/Adam_1%optimizer//policy/dense_1/kernel/Adam'optimizer//policy/dense_1/kernel/Adam_11optimizer//policy/main_graph_0/hidden_0/bias/Adam3optimizer//policy/main_graph_0/hidden_0/bias/Adam_13optimizer//policy/main_graph_0/hidden_0/kernel/Adam5optimizer//policy/main_graph_0/hidden_0/kernel/Adam_11optimizer//policy/main_graph_0/hidden_1/bias/Adam3optimizer//policy/main_graph_0/hidden_1/bias/Adam_13optimizer//policy/main_graph_0/hidden_1/kernel/Adam5optimizer//policy/main_graph_0/hidden_1/kernel/Adam_1optimizer/Variableoptimizer/Variable_1optimizer/Variable_2optimizer/beta1_poweroptimizer/beta1_power_1optimizer/beta2_poweroptimizer/beta2_power_1policy/dense/kernelpolicy/dense_1/kernel!policy/main_graph_0/hidden_0/bias#policy/main_graph_0/hidden_0/kernel!policy/main_graph_0/hidden_1/bias#policy/main_graph_0/hidden_1/kerneltrainer_major_versiontrainer_minor_versiontrainer_patch_versionversion_number*q
dtypesg
e2c
m
save_1/control_dependencyIdentitysave_1/Const^save_1/SaveV2*
T0*
_class
loc:@save_1/Const
�!
save_1/RestoreV2/tensor_namesConst"/device:CPU:0*
dtype0*� 
value� B� cBaction_output_shapeBglobal_stepBis_continuous_controlBmemory_sizeBoptimizer//curiosity_value/biasB!optimizer//curiosity_value/kernelB5optimizer//curiosity_vector_obs_encoder/hidden_0/biasB7optimizer//curiosity_vector_obs_encoder/hidden_0/kernelB5optimizer//curiosity_vector_obs_encoder/hidden_1/biasB7optimizer//curiosity_vector_obs_encoder/hidden_1/kernelBoptimizer//dense/biasBoptimizer//dense/kernelBoptimizer//dense_1/biasBoptimizer//dense_1/kernelBoptimizer//dense_2/biasBoptimizer//dense_2/kernelBoptimizer//dense_3/biasBoptimizer//dense_3/kernelBoptimizer//dense_4/biasBoptimizer//dense_4/kernelBoptimizer//extrinsic_value/biasB!optimizer//extrinsic_value/kernelB%optimizer//main_graph_0/hidden_0/biasB'optimizer//main_graph_0/hidden_0/kernelB%optimizer//main_graph_0/hidden_1/biasB'optimizer//main_graph_0/hidden_1/kernelB/optimizer//optimizer//curiosity_value/bias/AdamB1optimizer//optimizer//curiosity_value/bias/Adam_1B1optimizer//optimizer//curiosity_value/kernel/AdamB3optimizer//optimizer//curiosity_value/kernel/Adam_1BEoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/bias/AdamBGoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/bias/Adam_1BGoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/kernel/AdamBIoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/kernel/Adam_1BEoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/bias/AdamBGoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/bias/Adam_1BGoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/kernel/AdamBIoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/kernel/Adam_1B%optimizer//optimizer//dense/bias/AdamB'optimizer//optimizer//dense/bias/Adam_1B'optimizer//optimizer//dense/kernel/AdamB)optimizer//optimizer//dense/kernel/Adam_1B'optimizer//optimizer//dense_1/bias/AdamB)optimizer//optimizer//dense_1/bias/Adam_1B)optimizer//optimizer//dense_1/kernel/AdamB+optimizer//optimizer//dense_1/kernel/Adam_1B'optimizer//optimizer//dense_2/bias/AdamB)optimizer//optimizer//dense_2/bias/Adam_1B)optimizer//optimizer//dense_2/kernel/AdamB+optimizer//optimizer//dense_2/kernel/Adam_1B'optimizer//optimizer//dense_3/bias/AdamB)optimizer//optimizer//dense_3/bias/Adam_1B)optimizer//optimizer//dense_3/kernel/AdamB+optimizer//optimizer//dense_3/kernel/Adam_1B'optimizer//optimizer//dense_4/bias/AdamB)optimizer//optimizer//dense_4/bias/Adam_1B)optimizer//optimizer//dense_4/kernel/AdamB+optimizer//optimizer//dense_4/kernel/Adam_1B/optimizer//optimizer//extrinsic_value/bias/AdamB1optimizer//optimizer//extrinsic_value/bias/Adam_1B1optimizer//optimizer//extrinsic_value/kernel/AdamB3optimizer//optimizer//extrinsic_value/kernel/Adam_1B5optimizer//optimizer//main_graph_0/hidden_0/bias/AdamB7optimizer//optimizer//main_graph_0/hidden_0/bias/Adam_1B7optimizer//optimizer//main_graph_0/hidden_0/kernel/AdamB9optimizer//optimizer//main_graph_0/hidden_0/kernel/Adam_1B5optimizer//optimizer//main_graph_0/hidden_1/bias/AdamB7optimizer//optimizer//main_graph_0/hidden_1/bias/Adam_1B7optimizer//optimizer//main_graph_0/hidden_1/kernel/AdamB9optimizer//optimizer//main_graph_0/hidden_1/kernel/Adam_1B#optimizer//policy/dense/kernel/AdamB%optimizer//policy/dense/kernel/Adam_1B%optimizer//policy/dense_1/kernel/AdamB'optimizer//policy/dense_1/kernel/Adam_1B1optimizer//policy/main_graph_0/hidden_0/bias/AdamB3optimizer//policy/main_graph_0/hidden_0/bias/Adam_1B3optimizer//policy/main_graph_0/hidden_0/kernel/AdamB5optimizer//policy/main_graph_0/hidden_0/kernel/Adam_1B1optimizer//policy/main_graph_0/hidden_1/bias/AdamB3optimizer//policy/main_graph_0/hidden_1/bias/Adam_1B3optimizer//policy/main_graph_0/hidden_1/kernel/AdamB5optimizer//policy/main_graph_0/hidden_1/kernel/Adam_1Boptimizer/VariableBoptimizer/Variable_1Boptimizer/Variable_2Boptimizer/beta1_powerBoptimizer/beta1_power_1Boptimizer/beta2_powerBoptimizer/beta2_power_1Bpolicy/dense/kernelBpolicy/dense_1/kernelB!policy/main_graph_0/hidden_0/biasB#policy/main_graph_0/hidden_0/kernelB!policy/main_graph_0/hidden_1/biasB#policy/main_graph_0/hidden_1/kernelBtrainer_major_versionBtrainer_minor_versionBtrainer_patch_versionBversion_number
�
!save_1/RestoreV2/shape_and_slicesConst"/device:CPU:0*
dtype0*�
value�B�cB B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B B 
�
save_1/RestoreV2	RestoreV2save_1/Constsave_1/RestoreV2/tensor_names!save_1/RestoreV2/shape_and_slices"/device:CPU:0*q
dtypesg
e2c
�
save_1/AssignAssignaction_output_shapesave_1/RestoreV2*
T0*&
_class
loc:@action_output_shape*
use_locking(*
validate_shape(
�
save_1/Assign_1Assignglobal_stepsave_1/RestoreV2:1*
T0*
_class
loc:@global_step*
use_locking(*
validate_shape(
�
save_1/Assign_2Assignis_continuous_controlsave_1/RestoreV2:2*
T0*(
_class
loc:@is_continuous_control*
use_locking(*
validate_shape(
�
save_1/Assign_3Assignmemory_sizesave_1/RestoreV2:3*
T0*
_class
loc:@memory_size*
use_locking(*
validate_shape(
�
save_1/Assign_4Assignoptimizer//curiosity_value/biassave_1/RestoreV2:4*
T0*2
_class(
&$loc:@optimizer//curiosity_value/bias*
use_locking(*
validate_shape(
�
save_1/Assign_5Assign!optimizer//curiosity_value/kernelsave_1/RestoreV2:5*
T0*4
_class*
(&loc:@optimizer//curiosity_value/kernel*
use_locking(*
validate_shape(
�
save_1/Assign_6Assign5optimizer//curiosity_vector_obs_encoder/hidden_0/biassave_1/RestoreV2:6*
T0*H
_class>
<:loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/bias*
use_locking(*
validate_shape(
�
save_1/Assign_7Assign7optimizer//curiosity_vector_obs_encoder/hidden_0/kernelsave_1/RestoreV2:7*
T0*J
_class@
><loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/kernel*
use_locking(*
validate_shape(
�
save_1/Assign_8Assign5optimizer//curiosity_vector_obs_encoder/hidden_1/biassave_1/RestoreV2:8*
T0*H
_class>
<:loc:@optimizer//curiosity_vector_obs_encoder/hidden_1/bias*
use_locking(*
validate_shape(
�
save_1/Assign_9Assign7optimizer//curiosity_vector_obs_encoder/hidden_1/kernelsave_1/RestoreV2:9*
T0*J
_class@
><loc:@optimizer//curiosity_vector_obs_encoder/hidden_1/kernel*
use_locking(*
validate_shape(
�
save_1/Assign_10Assignoptimizer//dense/biassave_1/RestoreV2:10*
T0*(
_class
loc:@optimizer//dense/bias*
use_locking(*
validate_shape(
�
save_1/Assign_11Assignoptimizer//dense/kernelsave_1/RestoreV2:11*
T0**
_class 
loc:@optimizer//dense/kernel*
use_locking(*
validate_shape(
�
save_1/Assign_12Assignoptimizer//dense_1/biassave_1/RestoreV2:12*
T0**
_class 
loc:@optimizer//dense_1/bias*
use_locking(*
validate_shape(
�
save_1/Assign_13Assignoptimizer//dense_1/kernelsave_1/RestoreV2:13*
T0*,
_class"
 loc:@optimizer//dense_1/kernel*
use_locking(*
validate_shape(
�
save_1/Assign_14Assignoptimizer//dense_2/biassave_1/RestoreV2:14*
T0**
_class 
loc:@optimizer//dense_2/bias*
use_locking(*
validate_shape(
�
save_1/Assign_15Assignoptimizer//dense_2/kernelsave_1/RestoreV2:15*
T0*,
_class"
 loc:@optimizer//dense_2/kernel*
use_locking(*
validate_shape(
�
save_1/Assign_16Assignoptimizer//dense_3/biassave_1/RestoreV2:16*
T0**
_class 
loc:@optimizer//dense_3/bias*
use_locking(*
validate_shape(
�
save_1/Assign_17Assignoptimizer//dense_3/kernelsave_1/RestoreV2:17*
T0*,
_class"
 loc:@optimizer//dense_3/kernel*
use_locking(*
validate_shape(
�
save_1/Assign_18Assignoptimizer//dense_4/biassave_1/RestoreV2:18*
T0**
_class 
loc:@optimizer//dense_4/bias*
use_locking(*
validate_shape(
�
save_1/Assign_19Assignoptimizer//dense_4/kernelsave_1/RestoreV2:19*
T0*,
_class"
 loc:@optimizer//dense_4/kernel*
use_locking(*
validate_shape(
�
save_1/Assign_20Assignoptimizer//extrinsic_value/biassave_1/RestoreV2:20*
T0*2
_class(
&$loc:@optimizer//extrinsic_value/bias*
use_locking(*
validate_shape(
�
save_1/Assign_21Assign!optimizer//extrinsic_value/kernelsave_1/RestoreV2:21*
T0*4
_class*
(&loc:@optimizer//extrinsic_value/kernel*
use_locking(*
validate_shape(
�
save_1/Assign_22Assign%optimizer//main_graph_0/hidden_0/biassave_1/RestoreV2:22*
T0*8
_class.
,*loc:@optimizer//main_graph_0/hidden_0/bias*
use_locking(*
validate_shape(
�
save_1/Assign_23Assign'optimizer//main_graph_0/hidden_0/kernelsave_1/RestoreV2:23*
T0*:
_class0
.,loc:@optimizer//main_graph_0/hidden_0/kernel*
use_locking(*
validate_shape(
�
save_1/Assign_24Assign%optimizer//main_graph_0/hidden_1/biassave_1/RestoreV2:24*
T0*8
_class.
,*loc:@optimizer//main_graph_0/hidden_1/bias*
use_locking(*
validate_shape(
�
save_1/Assign_25Assign'optimizer//main_graph_0/hidden_1/kernelsave_1/RestoreV2:25*
T0*:
_class0
.,loc:@optimizer//main_graph_0/hidden_1/kernel*
use_locking(*
validate_shape(
�
save_1/Assign_26Assign/optimizer//optimizer//curiosity_value/bias/Adamsave_1/RestoreV2:26*
T0*2
_class(
&$loc:@optimizer//curiosity_value/bias*
use_locking(*
validate_shape(
�
save_1/Assign_27Assign1optimizer//optimizer//curiosity_value/bias/Adam_1save_1/RestoreV2:27*
T0*2
_class(
&$loc:@optimizer//curiosity_value/bias*
use_locking(*
validate_shape(
�
save_1/Assign_28Assign1optimizer//optimizer//curiosity_value/kernel/Adamsave_1/RestoreV2:28*
T0*4
_class*
(&loc:@optimizer//curiosity_value/kernel*
use_locking(*
validate_shape(
�
save_1/Assign_29Assign3optimizer//optimizer//curiosity_value/kernel/Adam_1save_1/RestoreV2:29*
T0*4
_class*
(&loc:@optimizer//curiosity_value/kernel*
use_locking(*
validate_shape(
�
save_1/Assign_30AssignEoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/bias/Adamsave_1/RestoreV2:30*
T0*H
_class>
<:loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/bias*
use_locking(*
validate_shape(
�
save_1/Assign_31AssignGoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/bias/Adam_1save_1/RestoreV2:31*
T0*H
_class>
<:loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/bias*
use_locking(*
validate_shape(
�
save_1/Assign_32AssignGoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/kernel/Adamsave_1/RestoreV2:32*
T0*J
_class@
><loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/kernel*
use_locking(*
validate_shape(
�
save_1/Assign_33AssignIoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/kernel/Adam_1save_1/RestoreV2:33*
T0*J
_class@
><loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/kernel*
use_locking(*
validate_shape(
�
save_1/Assign_34AssignEoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/bias/Adamsave_1/RestoreV2:34*
T0*H
_class>
<:loc:@optimizer//curiosity_vector_obs_encoder/hidden_1/bias*
use_locking(*
validate_shape(
�
save_1/Assign_35AssignGoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/bias/Adam_1save_1/RestoreV2:35*
T0*H
_class>
<:loc:@optimizer//curiosity_vector_obs_encoder/hidden_1/bias*
use_locking(*
validate_shape(
�
save_1/Assign_36AssignGoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/kernel/Adamsave_1/RestoreV2:36*
T0*J
_class@
><loc:@optimizer//curiosity_vector_obs_encoder/hidden_1/kernel*
use_locking(*
validate_shape(
�
save_1/Assign_37AssignIoptimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/kernel/Adam_1save_1/RestoreV2:37*
T0*J
_class@
><loc:@optimizer//curiosity_vector_obs_encoder/hidden_1/kernel*
use_locking(*
validate_shape(
�
save_1/Assign_38Assign%optimizer//optimizer//dense/bias/Adamsave_1/RestoreV2:38*
T0*(
_class
loc:@optimizer//dense/bias*
use_locking(*
validate_shape(
�
save_1/Assign_39Assign'optimizer//optimizer//dense/bias/Adam_1save_1/RestoreV2:39*
T0*(
_class
loc:@optimizer//dense/bias*
use_locking(*
validate_shape(
�
save_1/Assign_40Assign'optimizer//optimizer//dense/kernel/Adamsave_1/RestoreV2:40*
T0**
_class 
loc:@optimizer//dense/kernel*
use_locking(*
validate_shape(
�
save_1/Assign_41Assign)optimizer//optimizer//dense/kernel/Adam_1save_1/RestoreV2:41*
T0**
_class 
loc:@optimizer//dense/kernel*
use_locking(*
validate_shape(
�
save_1/Assign_42Assign'optimizer//optimizer//dense_1/bias/Adamsave_1/RestoreV2:42*
T0**
_class 
loc:@optimizer//dense_1/bias*
use_locking(*
validate_shape(
�
save_1/Assign_43Assign)optimizer//optimizer//dense_1/bias/Adam_1save_1/RestoreV2:43*
T0**
_class 
loc:@optimizer//dense_1/bias*
use_locking(*
validate_shape(
�
save_1/Assign_44Assign)optimizer//optimizer//dense_1/kernel/Adamsave_1/RestoreV2:44*
T0*,
_class"
 loc:@optimizer//dense_1/kernel*
use_locking(*
validate_shape(
�
save_1/Assign_45Assign+optimizer//optimizer//dense_1/kernel/Adam_1save_1/RestoreV2:45*
T0*,
_class"
 loc:@optimizer//dense_1/kernel*
use_locking(*
validate_shape(
�
save_1/Assign_46Assign'optimizer//optimizer//dense_2/bias/Adamsave_1/RestoreV2:46*
T0**
_class 
loc:@optimizer//dense_2/bias*
use_locking(*
validate_shape(
�
save_1/Assign_47Assign)optimizer//optimizer//dense_2/bias/Adam_1save_1/RestoreV2:47*
T0**
_class 
loc:@optimizer//dense_2/bias*
use_locking(*
validate_shape(
�
save_1/Assign_48Assign)optimizer//optimizer//dense_2/kernel/Adamsave_1/RestoreV2:48*
T0*,
_class"
 loc:@optimizer//dense_2/kernel*
use_locking(*
validate_shape(
�
save_1/Assign_49Assign+optimizer//optimizer//dense_2/kernel/Adam_1save_1/RestoreV2:49*
T0*,
_class"
 loc:@optimizer//dense_2/kernel*
use_locking(*
validate_shape(
�
save_1/Assign_50Assign'optimizer//optimizer//dense_3/bias/Adamsave_1/RestoreV2:50*
T0**
_class 
loc:@optimizer//dense_3/bias*
use_locking(*
validate_shape(
�
save_1/Assign_51Assign)optimizer//optimizer//dense_3/bias/Adam_1save_1/RestoreV2:51*
T0**
_class 
loc:@optimizer//dense_3/bias*
use_locking(*
validate_shape(
�
save_1/Assign_52Assign)optimizer//optimizer//dense_3/kernel/Adamsave_1/RestoreV2:52*
T0*,
_class"
 loc:@optimizer//dense_3/kernel*
use_locking(*
validate_shape(
�
save_1/Assign_53Assign+optimizer//optimizer//dense_3/kernel/Adam_1save_1/RestoreV2:53*
T0*,
_class"
 loc:@optimizer//dense_3/kernel*
use_locking(*
validate_shape(
�
save_1/Assign_54Assign'optimizer//optimizer//dense_4/bias/Adamsave_1/RestoreV2:54*
T0**
_class 
loc:@optimizer//dense_4/bias*
use_locking(*
validate_shape(
�
save_1/Assign_55Assign)optimizer//optimizer//dense_4/bias/Adam_1save_1/RestoreV2:55*
T0**
_class 
loc:@optimizer//dense_4/bias*
use_locking(*
validate_shape(
�
save_1/Assign_56Assign)optimizer//optimizer//dense_4/kernel/Adamsave_1/RestoreV2:56*
T0*,
_class"
 loc:@optimizer//dense_4/kernel*
use_locking(*
validate_shape(
�
save_1/Assign_57Assign+optimizer//optimizer//dense_4/kernel/Adam_1save_1/RestoreV2:57*
T0*,
_class"
 loc:@optimizer//dense_4/kernel*
use_locking(*
validate_shape(
�
save_1/Assign_58Assign/optimizer//optimizer//extrinsic_value/bias/Adamsave_1/RestoreV2:58*
T0*2
_class(
&$loc:@optimizer//extrinsic_value/bias*
use_locking(*
validate_shape(
�
save_1/Assign_59Assign1optimizer//optimizer//extrinsic_value/bias/Adam_1save_1/RestoreV2:59*
T0*2
_class(
&$loc:@optimizer//extrinsic_value/bias*
use_locking(*
validate_shape(
�
save_1/Assign_60Assign1optimizer//optimizer//extrinsic_value/kernel/Adamsave_1/RestoreV2:60*
T0*4
_class*
(&loc:@optimizer//extrinsic_value/kernel*
use_locking(*
validate_shape(
�
save_1/Assign_61Assign3optimizer//optimizer//extrinsic_value/kernel/Adam_1save_1/RestoreV2:61*
T0*4
_class*
(&loc:@optimizer//extrinsic_value/kernel*
use_locking(*
validate_shape(
�
save_1/Assign_62Assign5optimizer//optimizer//main_graph_0/hidden_0/bias/Adamsave_1/RestoreV2:62*
T0*8
_class.
,*loc:@optimizer//main_graph_0/hidden_0/bias*
use_locking(*
validate_shape(
�
save_1/Assign_63Assign7optimizer//optimizer//main_graph_0/hidden_0/bias/Adam_1save_1/RestoreV2:63*
T0*8
_class.
,*loc:@optimizer//main_graph_0/hidden_0/bias*
use_locking(*
validate_shape(
�
save_1/Assign_64Assign7optimizer//optimizer//main_graph_0/hidden_0/kernel/Adamsave_1/RestoreV2:64*
T0*:
_class0
.,loc:@optimizer//main_graph_0/hidden_0/kernel*
use_locking(*
validate_shape(
�
save_1/Assign_65Assign9optimizer//optimizer//main_graph_0/hidden_0/kernel/Adam_1save_1/RestoreV2:65*
T0*:
_class0
.,loc:@optimizer//main_graph_0/hidden_0/kernel*
use_locking(*
validate_shape(
�
save_1/Assign_66Assign5optimizer//optimizer//main_graph_0/hidden_1/bias/Adamsave_1/RestoreV2:66*
T0*8
_class.
,*loc:@optimizer//main_graph_0/hidden_1/bias*
use_locking(*
validate_shape(
�
save_1/Assign_67Assign7optimizer//optimizer//main_graph_0/hidden_1/bias/Adam_1save_1/RestoreV2:67*
T0*8
_class.
,*loc:@optimizer//main_graph_0/hidden_1/bias*
use_locking(*
validate_shape(
�
save_1/Assign_68Assign7optimizer//optimizer//main_graph_0/hidden_1/kernel/Adamsave_1/RestoreV2:68*
T0*:
_class0
.,loc:@optimizer//main_graph_0/hidden_1/kernel*
use_locking(*
validate_shape(
�
save_1/Assign_69Assign9optimizer//optimizer//main_graph_0/hidden_1/kernel/Adam_1save_1/RestoreV2:69*
T0*:
_class0
.,loc:@optimizer//main_graph_0/hidden_1/kernel*
use_locking(*
validate_shape(
�
save_1/Assign_70Assign#optimizer//policy/dense/kernel/Adamsave_1/RestoreV2:70*
T0*&
_class
loc:@policy/dense/kernel*
use_locking(*
validate_shape(
�
save_1/Assign_71Assign%optimizer//policy/dense/kernel/Adam_1save_1/RestoreV2:71*
T0*&
_class
loc:@policy/dense/kernel*
use_locking(*
validate_shape(
�
save_1/Assign_72Assign%optimizer//policy/dense_1/kernel/Adamsave_1/RestoreV2:72*
T0*(
_class
loc:@policy/dense_1/kernel*
use_locking(*
validate_shape(
�
save_1/Assign_73Assign'optimizer//policy/dense_1/kernel/Adam_1save_1/RestoreV2:73*
T0*(
_class
loc:@policy/dense_1/kernel*
use_locking(*
validate_shape(
�
save_1/Assign_74Assign1optimizer//policy/main_graph_0/hidden_0/bias/Adamsave_1/RestoreV2:74*
T0*4
_class*
(&loc:@policy/main_graph_0/hidden_0/bias*
use_locking(*
validate_shape(
�
save_1/Assign_75Assign3optimizer//policy/main_graph_0/hidden_0/bias/Adam_1save_1/RestoreV2:75*
T0*4
_class*
(&loc:@policy/main_graph_0/hidden_0/bias*
use_locking(*
validate_shape(
�
save_1/Assign_76Assign3optimizer//policy/main_graph_0/hidden_0/kernel/Adamsave_1/RestoreV2:76*
T0*6
_class,
*(loc:@policy/main_graph_0/hidden_0/kernel*
use_locking(*
validate_shape(
�
save_1/Assign_77Assign5optimizer//policy/main_graph_0/hidden_0/kernel/Adam_1save_1/RestoreV2:77*
T0*6
_class,
*(loc:@policy/main_graph_0/hidden_0/kernel*
use_locking(*
validate_shape(
�
save_1/Assign_78Assign1optimizer//policy/main_graph_0/hidden_1/bias/Adamsave_1/RestoreV2:78*
T0*4
_class*
(&loc:@policy/main_graph_0/hidden_1/bias*
use_locking(*
validate_shape(
�
save_1/Assign_79Assign3optimizer//policy/main_graph_0/hidden_1/bias/Adam_1save_1/RestoreV2:79*
T0*4
_class*
(&loc:@policy/main_graph_0/hidden_1/bias*
use_locking(*
validate_shape(
�
save_1/Assign_80Assign3optimizer//policy/main_graph_0/hidden_1/kernel/Adamsave_1/RestoreV2:80*
T0*6
_class,
*(loc:@policy/main_graph_0/hidden_1/kernel*
use_locking(*
validate_shape(
�
save_1/Assign_81Assign5optimizer//policy/main_graph_0/hidden_1/kernel/Adam_1save_1/RestoreV2:81*
T0*6
_class,
*(loc:@policy/main_graph_0/hidden_1/kernel*
use_locking(*
validate_shape(
�
save_1/Assign_82Assignoptimizer/Variablesave_1/RestoreV2:82*
T0*%
_class
loc:@optimizer/Variable*
use_locking(*
validate_shape(
�
save_1/Assign_83Assignoptimizer/Variable_1save_1/RestoreV2:83*
T0*'
_class
loc:@optimizer/Variable_1*
use_locking(*
validate_shape(
�
save_1/Assign_84Assignoptimizer/Variable_2save_1/RestoreV2:84*
T0*'
_class
loc:@optimizer/Variable_2*
use_locking(*
validate_shape(
�
save_1/Assign_85Assignoptimizer/beta1_powersave_1/RestoreV2:85*
T0*H
_class>
<:loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/bias*
use_locking(*
validate_shape(
�
save_1/Assign_86Assignoptimizer/beta1_power_1save_1/RestoreV2:86*
T0*2
_class(
&$loc:@optimizer//curiosity_value/bias*
use_locking(*
validate_shape(
�
save_1/Assign_87Assignoptimizer/beta2_powersave_1/RestoreV2:87*
T0*H
_class>
<:loc:@optimizer//curiosity_vector_obs_encoder/hidden_0/bias*
use_locking(*
validate_shape(
�
save_1/Assign_88Assignoptimizer/beta2_power_1save_1/RestoreV2:88*
T0*2
_class(
&$loc:@optimizer//curiosity_value/bias*
use_locking(*
validate_shape(
�
save_1/Assign_89Assignpolicy/dense/kernelsave_1/RestoreV2:89*
T0*&
_class
loc:@policy/dense/kernel*
use_locking(*
validate_shape(
�
save_1/Assign_90Assignpolicy/dense_1/kernelsave_1/RestoreV2:90*
T0*(
_class
loc:@policy/dense_1/kernel*
use_locking(*
validate_shape(
�
save_1/Assign_91Assign!policy/main_graph_0/hidden_0/biassave_1/RestoreV2:91*
T0*4
_class*
(&loc:@policy/main_graph_0/hidden_0/bias*
use_locking(*
validate_shape(
�
save_1/Assign_92Assign#policy/main_graph_0/hidden_0/kernelsave_1/RestoreV2:92*
T0*6
_class,
*(loc:@policy/main_graph_0/hidden_0/kernel*
use_locking(*
validate_shape(
�
save_1/Assign_93Assign!policy/main_graph_0/hidden_1/biassave_1/RestoreV2:93*
T0*4
_class*
(&loc:@policy/main_graph_0/hidden_1/bias*
use_locking(*
validate_shape(
�
save_1/Assign_94Assign#policy/main_graph_0/hidden_1/kernelsave_1/RestoreV2:94*
T0*6
_class,
*(loc:@policy/main_graph_0/hidden_1/kernel*
use_locking(*
validate_shape(
�
save_1/Assign_95Assigntrainer_major_versionsave_1/RestoreV2:95*
T0*(
_class
loc:@trainer_major_version*
use_locking(*
validate_shape(
�
save_1/Assign_96Assigntrainer_minor_versionsave_1/RestoreV2:96*
T0*(
_class
loc:@trainer_minor_version*
use_locking(*
validate_shape(
�
save_1/Assign_97Assigntrainer_patch_versionsave_1/RestoreV2:97*
T0*(
_class
loc:@trainer_patch_version*
use_locking(*
validate_shape(
�
save_1/Assign_98Assignversion_numbersave_1/RestoreV2:98*
T0*!
_class
loc:@version_number*
use_locking(*
validate_shape(
�
save_1/restore_allNoOp^save_1/Assign^save_1/Assign_1^save_1/Assign_10^save_1/Assign_11^save_1/Assign_12^save_1/Assign_13^save_1/Assign_14^save_1/Assign_15^save_1/Assign_16^save_1/Assign_17^save_1/Assign_18^save_1/Assign_19^save_1/Assign_2^save_1/Assign_20^save_1/Assign_21^save_1/Assign_22^save_1/Assign_23^save_1/Assign_24^save_1/Assign_25^save_1/Assign_26^save_1/Assign_27^save_1/Assign_28^save_1/Assign_29^save_1/Assign_3^save_1/Assign_30^save_1/Assign_31^save_1/Assign_32^save_1/Assign_33^save_1/Assign_34^save_1/Assign_35^save_1/Assign_36^save_1/Assign_37^save_1/Assign_38^save_1/Assign_39^save_1/Assign_4^save_1/Assign_40^save_1/Assign_41^save_1/Assign_42^save_1/Assign_43^save_1/Assign_44^save_1/Assign_45^save_1/Assign_46^save_1/Assign_47^save_1/Assign_48^save_1/Assign_49^save_1/Assign_5^save_1/Assign_50^save_1/Assign_51^save_1/Assign_52^save_1/Assign_53^save_1/Assign_54^save_1/Assign_55^save_1/Assign_56^save_1/Assign_57^save_1/Assign_58^save_1/Assign_59^save_1/Assign_6^save_1/Assign_60^save_1/Assign_61^save_1/Assign_62^save_1/Assign_63^save_1/Assign_64^save_1/Assign_65^save_1/Assign_66^save_1/Assign_67^save_1/Assign_68^save_1/Assign_69^save_1/Assign_7^save_1/Assign_70^save_1/Assign_71^save_1/Assign_72^save_1/Assign_73^save_1/Assign_74^save_1/Assign_75^save_1/Assign_76^save_1/Assign_77^save_1/Assign_78^save_1/Assign_79^save_1/Assign_8^save_1/Assign_80^save_1/Assign_81^save_1/Assign_82^save_1/Assign_83^save_1/Assign_84^save_1/Assign_85^save_1/Assign_86^save_1/Assign_87^save_1/Assign_88^save_1/Assign_89^save_1/Assign_9^save_1/Assign_90^save_1/Assign_91^save_1/Assign_92^save_1/Assign_93^save_1/Assign_94^save_1/Assign_95^save_1/Assign_96^save_1/Assign_97^save_1/Assign_98
�&
init_1NoOp^action_output_shape/Assign^global_step/Assign^is_continuous_control/Assign^memory_size/Assign'^optimizer//curiosity_value/bias/Assign)^optimizer//curiosity_value/kernel/Assign=^optimizer//curiosity_vector_obs_encoder/hidden_0/bias/Assign?^optimizer//curiosity_vector_obs_encoder/hidden_0/kernel/Assign=^optimizer//curiosity_vector_obs_encoder/hidden_1/bias/Assign?^optimizer//curiosity_vector_obs_encoder/hidden_1/kernel/Assign^optimizer//dense/bias/Assign^optimizer//dense/kernel/Assign^optimizer//dense_1/bias/Assign!^optimizer//dense_1/kernel/Assign^optimizer//dense_2/bias/Assign!^optimizer//dense_2/kernel/Assign^optimizer//dense_3/bias/Assign!^optimizer//dense_3/kernel/Assign^optimizer//dense_4/bias/Assign!^optimizer//dense_4/kernel/Assign'^optimizer//extrinsic_value/bias/Assign)^optimizer//extrinsic_value/kernel/Assign-^optimizer//main_graph_0/hidden_0/bias/Assign/^optimizer//main_graph_0/hidden_0/kernel/Assign-^optimizer//main_graph_0/hidden_1/bias/Assign/^optimizer//main_graph_0/hidden_1/kernel/Assign7^optimizer//optimizer//curiosity_value/bias/Adam/Assign9^optimizer//optimizer//curiosity_value/bias/Adam_1/Assign9^optimizer//optimizer//curiosity_value/kernel/Adam/Assign;^optimizer//optimizer//curiosity_value/kernel/Adam_1/AssignM^optimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/bias/Adam/AssignO^optimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/bias/Adam_1/AssignO^optimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/kernel/Adam/AssignQ^optimizer//optimizer//curiosity_vector_obs_encoder/hidden_0/kernel/Adam_1/AssignM^optimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/bias/Adam/AssignO^optimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/bias/Adam_1/AssignO^optimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/kernel/Adam/AssignQ^optimizer//optimizer//curiosity_vector_obs_encoder/hidden_1/kernel/Adam_1/Assign-^optimizer//optimizer//dense/bias/Adam/Assign/^optimizer//optimizer//dense/bias/Adam_1/Assign/^optimizer//optimizer//dense/kernel/Adam/Assign1^optimizer//optimizer//dense/kernel/Adam_1/Assign/^optimizer//optimizer//dense_1/bias/Adam/Assign1^optimizer//optimizer//dense_1/bias/Adam_1/Assign1^optimizer//optimizer//dense_1/kernel/Adam/Assign3^optimizer//optimizer//dense_1/kernel/Adam_1/Assign/^optimizer//optimizer//dense_2/bias/Adam/Assign1^optimizer//optimizer//dense_2/bias/Adam_1/Assign1^optimizer//optimizer//dense_2/kernel/Adam/Assign3^optimizer//optimizer//dense_2/kernel/Adam_1/Assign/^optimizer//optimizer//dense_3/bias/Adam/Assign1^optimizer//optimizer//dense_3/bias/Adam_1/Assign1^optimizer//optimizer//dense_3/kernel/Adam/Assign3^optimizer//optimizer//dense_3/kernel/Adam_1/Assign/^optimizer//optimizer//dense_4/bias/Adam/Assign1^optimizer//optimizer//dense_4/bias/Adam_1/Assign1^optimizer//optimizer//dense_4/kernel/Adam/Assign3^optimizer//optimizer//dense_4/kernel/Adam_1/Assign7^optimizer//optimizer//extrinsic_value/bias/Adam/Assign9^optimizer//optimizer//extrinsic_value/bias/Adam_1/Assign9^optimizer//optimizer//extrinsic_value/kernel/Adam/Assign;^optimizer//optimizer//extrinsic_value/kernel/Adam_1/Assign=^optimizer//optimizer//main_graph_0/hidden_0/bias/Adam/Assign?^optimizer//optimizer//main_graph_0/hidden_0/bias/Adam_1/Assign?^optimizer//optimizer//main_graph_0/hidden_0/kernel/Adam/AssignA^optimizer//optimizer//main_graph_0/hidden_0/kernel/Adam_1/Assign=^optimizer//optimizer//main_graph_0/hidden_1/bias/Adam/Assign?^optimizer//optimizer//main_graph_0/hidden_1/bias/Adam_1/Assign?^optimizer//optimizer//main_graph_0/hidden_1/kernel/Adam/AssignA^optimizer//optimizer//main_graph_0/hidden_1/kernel/Adam_1/Assign+^optimizer//policy/dense/kernel/Adam/Assign-^optimizer//policy/dense/kernel/Adam_1/Assign-^optimizer//policy/dense_1/kernel/Adam/Assign/^optimizer//policy/dense_1/kernel/Adam_1/Assign9^optimizer//policy/main_graph_0/hidden_0/bias/Adam/Assign;^optimizer//policy/main_graph_0/hidden_0/bias/Adam_1/Assign;^optimizer//policy/main_graph_0/hidden_0/kernel/Adam/Assign=^optimizer//policy/main_graph_0/hidden_0/kernel/Adam_1/Assign9^optimizer//policy/main_graph_0/hidden_1/bias/Adam/Assign;^optimizer//policy/main_graph_0/hidden_1/bias/Adam_1/Assign;^optimizer//policy/main_graph_0/hidden_1/kernel/Adam/Assign=^optimizer//policy/main_graph_0/hidden_1/kernel/Adam_1/Assign^optimizer/Variable/Assign^optimizer/Variable_1/Assign^optimizer/Variable_2/Assign^optimizer/beta1_power/Assign^optimizer/beta1_power_1/Assign^optimizer/beta2_power/Assign^optimizer/beta2_power_1/Assign^policy/dense/kernel/Assign^policy/dense_1/kernel/Assign)^policy/main_graph_0/hidden_0/bias/Assign+^policy/main_graph_0/hidden_0/kernel/Assign)^policy/main_graph_0/hidden_1/bias/Assign+^policy/main_graph_0/hidden_1/kernel/Assign^trainer_major_version/Assign^trainer_minor_version/Assign^trainer_patch_version/Assign^version_number/Assign"�