import os
import random
import time
from subprocess import call

# 画像ディレクトリと画像ファイルの一覧
#image_directory = "noise_images"  # 作成した画像のフォルダ
image_directory = "my_image"  # 作成した画像のフォルダ
image_files = [f for f in os.listdir(image_directory) if f.endswith('.png')]

# Gazeboモデルのテクスチャを動的に変更する関数
def update_gazebo_texture(image_path):
    # テクスチャファイルのパス
    texture_file_path = "/home/ciero/catkin_ws/src/nav_cloning/models/changing_model2/models/colorfull_box/materials/scripts/textures/my_texture.png"
    # マテリアルファイルのパス
    material_file_path = "/home/ciero/catkin_ws/src/nav_cloning/models/changing_model2/models/colorfull_box/materials/scripts/colorfull_box.material"

    # 画像をコピー
    call(["cp", image_path, texture_file_path])  
    print(f"Updated texture to {image_path}")

    # マテリアルファイルの更新
    with open(material_file_path, 'w') as f:
        f.write(f"""material colorfull_boxTexture
{{
  technique
  {{
    pass
    {{
      texture_unit
      {{
        texture textures/my_texture.png
        scale 1 1
      }}
    }}
  }}
}}""")
    print(f"Updated material file with new texture path: {material_file_path}")


# 画像をランダムに選んで、指定時間ごとに変更
def cycle_images(interval_seconds):
    while True:
        # ランダムに画像を選択
        selected_image = random.choice(image_files)
        image_path = os.path.join(image_directory, selected_image)

        # Gazeboモデルのテクスチャを更新
        update_gazebo_texture(image_path)

        # 指定した時間だけ待機
        time.sleep(interval_seconds)

# 一定時間ごとに画像を入れ替え（5秒ごとに変更）
#cycle_images(5)
cycle_images(0.5)

