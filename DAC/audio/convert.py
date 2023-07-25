#! /usr/bin/env nix-shell
#! nix-shell -i python3.11 -p python311
import wave, sys

if __name__ == "__main__":
	w = wave.open(sys.argv[1], "rb")
	frames = w.readframes(w.getnframes())
	print("#ifndef __BRRRRP_H")
	print("#define __BRRRRP_H")
	print()
	print("uint8_t buzzer[{}] = {{".format(len(frames)), end='')
	for n,f in enumerate(frames):
		if n > 0:
			print(",",f,end='')
		else:
			print(f,end='')
	print("};")
	print()
	print("#endif")
	w.close()
