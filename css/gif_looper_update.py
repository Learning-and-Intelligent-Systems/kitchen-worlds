## create a script that converts all gif images in the gifs folder to loop automatically
from os.path import join, abspath
from os import listdir

GIF_PATH = join('..', 'gifs')
GIFSICLE_PATH = '/Users/z/Documents/simulators/_tools/gifsicle/src'

if __name__ == 'main':
  gif_files = [abspath(join(GIF_PATH, f)) for f in listdir(GIF_PATH)]
  with open('gif_looper.sh', 'w') as f:
      f.write(f'cd {GIFSICLE_PATH}\n')
      for gif in gif_files:
          f.write(f'./gifsicle -bl {gif}\n')
