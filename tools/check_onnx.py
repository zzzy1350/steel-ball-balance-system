import onnx

model_path = "k230_artifacts/steel_ball_yolov8n_320.onnx"

model = onnx.load(model_path)

print("ONNX模型检查通过")

print("\n输入：")
for item in model.graph.input:
    shape = []
    for dim in item.type.tensor_type.shape.dim:
        if dim.dim_value:
            shape.append(dim.dim_value)
        else:
            shape.append(dim.dim_param)
    print(item.name, shape)

print("\n输出：")
for item in model.graph.output:
    shape = []
    for dim in item.type.tensor_type.shape.dim:
        if dim.dim_value:
            shape.append(dim.dim_value)
        else:
            shape.append(dim.dim_param)
    print(item.name, shape)