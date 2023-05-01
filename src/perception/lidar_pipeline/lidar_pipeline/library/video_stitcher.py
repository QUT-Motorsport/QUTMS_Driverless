import os

import moviepy.video.io.ImageSequenceClip


def stitch_figures(session_path, output_path):
    directories = []
    for file in os.listdir(session_path):
        fig_dir = os.path.join(session_path, file)
        if os.path.isdir(fig_dir):
            directories.append(fig_dir)

    image_sequences = []
    for image in os.listdir(directories[0]):
        image_sequences.append([directories[0] + "/" + image])

    for dir in directories[1:]:
        for i, image in enumerate(os.listdir(dir)):
            print(i, image)
            image_sequences[i].append(dir + "/" + image)

    print(image_sequences)
    for i, image_sequence in enumerate(image_sequences):
        clip = moviepy.video.io.ImageSequenceClip.ImageSequenceClip(image_sequence, fps=10)
        clip.write_videofile(f"{output_path}/{i}.mp4")
