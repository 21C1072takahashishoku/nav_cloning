import cupy
import importlib.util

print("CuPy version:", cupy.__version__)
print("CUDA available:", cupy.is_available())
print("CUDA Device:", cupy.cuda.Device())

# cuDNN のインポートチェック
spec = importlib.util.find_spec("cupy.cuda.cudnn")
if spec is not None:
    try:
        import cupy.cuda.cudnn as cudnn
        print("cuDNN version:", cudnn.getVersion())
        print("cuDNN loaded successfully.")
    except Exception as e:
        print("cuDNN import failed:", e)
else:
    print("cuDNN module not found in CuPy.")

