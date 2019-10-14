BOARD = nodemcuv2
BUILD_DIR = build
BUILD_EXTRA_FLAGS = -DASYNC_TCP_SSL_ENABLED=1

include /usr/share/arduino/makeEspArduino.mk

list_ota:
	echo `((avahi-browse _arduino._tcp --resolve --parsable --terminate) 2>/dev/null | grep -F "=;") | cut -d\; -f8`

monitor:
	minicom -D ${UPLOAD_PORT}

