import argparse
from datetime import datetime
import os, sys, subprocess, glob

parser = argparse.ArgumentParser()
parser.add_argument("-n", "--name", type=str, default="video_st_sb")
args = parser.parse_args()

script_path = os.path.abspath(__file__)
script_dir = os.path.dirname(script_path)
picture_dir = os.path.join(script_dir, "../pictures")
video_dir = os.path.join(script_dir, "../videos")

vname = datetime.now().strftime("video_%Y-%m-%d-%H-%M-%S_") + args.name + ".mp4"

command = [
  "ffmpeg", 
  "-r", "30", "-i", os.path.join(picture_dir, args.name) + "%08d.png", 
  "-r", "30", os.path.join(video_dir, vname),
]

try:
  subprocess.run(command, check=True)
  print(f"Video exported successfully: {os.path.join(video_dir, vname)}")
except Exception as e:
  print("Error occurred while running ffmpeg.")
  sys.exit(1)

try:
  del_pictures = glob.glob(os.path.join(picture_dir, args.name) + "*.png")
  for picture in del_pictures:
    os.remove(picture)
  print("Pictures cleaned up successfully.")
except Exception as e:
  print("Error occurred while cleaning up pictures.")
  sys.exit(1)