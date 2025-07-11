FROM pytorch/pytorch:2.4.0-cuda12.1-cudnn8-runtime

RUN apt-get update && apt-get install -y git ffmpeg libsm6 libxext6 && rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY . /app

RUN pip install --upgrade pip
RUN pip install -e .

CMD ["python", "app.py"]
