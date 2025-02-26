import torch

weights = "yolov5s.pt"
model = torch.load(weights, map_location=torch.device('cpu'))['model'].float()

model.eval()
# model.model[-1].export = True
input_data = torch.randn(1, 3, 416, 416)#(高，宽)

y = model(input_data)
print("model = ", model)
torch.onnx.export(model, input_data, "yolov5s.onnx", verbose=False, opset_version=11,
                  input_names=['images'], output_names=['classes', 'boxes'] if y is None else ['output'])
print("Done!")