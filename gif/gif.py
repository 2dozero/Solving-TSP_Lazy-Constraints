import os
import re
import imageio
# 파일 이름에서 숫자를 추출하여 정수로 변환하는 함수
def extract_number(file_name):
    match = re.search(r'\d+', file_name)
    if match:
        return int(match.group())
    return 0
def create_gif_from_pngs(directory_path, output_filename, duration):
    """
    주어진 디렉토리에서 모든 PNG 이미지를 찾아 GIF로 변환합니다.
    :param directory_path: PNG 이미지들이 있는 디렉토리의 경로
    :param output_filename: 생성될 GIF 파일의 이름
    :param duration: GIF 내에서 각 이미지가 보여지는 시간 (초 단위)
    """
    # 주어진 디렉토리에서 모든 파일의 목록을 가져옵니다.
    files = os.listdir(directory_path)
    # 파일 목록에서 PNG 이미지만 필터링합니다.
    png_files = [f for f in files if f.endswith('.png')]
    # PNG 파일들을 경로와 함께 전체 경로로 변환합니다.
    png_files = [os.path.join(directory_path, f) for f in png_files]
    # 파일 이름 기준으로 정렬합니다.
    # 파일 목록을 숫자 기준으로 정렬
    sorted_files = sorted(png_files, key=extract_number)
    print(sorted_files)
    with imageio.get_writer(output_filename, mode='I', duration=duration) as writer:
        for filename in sorted_files:
            image = imageio.imread(filename)
            writer.append_data(image)
# 사용 예
directory_path = './image/' # 현재 디렉토리를 사용
output_filename = './gif/output.gif'
duration = 0.5 # 각 이미지가 0.5초 동안 보여짐
create_gif_from_pngs(directory_path, output_filename, duration)