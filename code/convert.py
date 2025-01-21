import os
from pydub import AudioSegment
from multiprocessing import Pool, cpu_count

def convert_file(args):
    ogg_path, mp3_path = args
    try:
        audio = AudioSegment.from_ogg(ogg_path)
        audio.export(mp3_path, format="mp3")
        print(f"Converted: {os.path.basename(ogg_path)} -> {os.path.basename(mp3_path)}")
    except Exception as e:
        print(f"Failed to convert {os.path.basename(ogg_path)}: {e}")

def convert_ogg_to_mp3_multi(source_folder, target_folder):
    if not os.path.exists(target_folder):
        os.makedirs(target_folder)

    tasks = []

    for file_name in os.listdir(source_folder):
        if file_name.endswith(".ogg"):
            ogg_path = os.path.join(source_folder, file_name)
            mp3_file_name = os.path.splitext(file_name)[0] + ".mp3"
            mp3_path = os.path.join(target_folder, mp3_file_name)
            tasks.append((ogg_path, mp3_path))

    with Pool(cpu_count()) as pool:
        pool.map(convert_file, tasks)

if __name__ == "__main__":
    source_folder = r"C:\Users\Nack\Desktop\Song"
    target_folder = os.path.join(source_folder, "Converted")

    convert_ogg_to_mp3_multi(source_folder, target_folder)
    print(f"Conversion complete. MP3 files saved to: {target_folder}")
