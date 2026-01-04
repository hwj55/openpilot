import sys
import os
if __name__ == "__main__":
    font_dir = "selfdrive/assets/fonts"
    for f in ["Audiowide-Regular", "Inter-SemiBold", "JetBrainsMono-Medium"]:
        open(f"{font_dir}/{f}.fnt", 'a').close()
        open(f"{font_dir}/{f}.png", 'a').close()
    sys.exit(0)
