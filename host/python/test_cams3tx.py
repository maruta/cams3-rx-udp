import pygame
import cv2
import numpy as np
from cams3tx import CamS3TX

WIDTH = 176
HEIGHT = 144
SCALE = 4
WINDOW_WIDTH = WIDTH * SCALE
WINDOW_HEIGHT = HEIGHT * SCALE

def display_image(screen, img):
    img = cv2.resize(img, (WINDOW_WIDTH, WINDOW_HEIGHT), interpolation=cv2.INTER_NEAREST)
    img_surface = pygame.surfarray.make_surface(img.transpose(1, 0, 2))
    screen.blit(img_surface, (0, 0))
    pygame.display.update()

def main():
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
    pygame.display.set_caption("CamS3TX Received Image")

    with CamS3TX() as cam:
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            image = cam.get_image()
            if image is not None:
                display_image(screen, image)
                avg_rtt = cam.get_rtt_stat()
                avg_frame_dt = cam.get_frame_dt()
                info = ""
                if avg_rtt is not None:
                    info += f"RTT: {avg_rtt*1000:.1f} ms"
                if avg_frame_dt is not None:
                    info += f", frame DT: {avg_frame_dt*1000:.1f} ms"
                print(info)

            cam.send_servo_command([1500, 1500])



if __name__ == "__main__":
    main()