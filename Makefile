include $(shell rospack find mk)/cmake.mk

distclean:
# 	make clean
	cd collabnav_gmapping && make clean && rm -fr bin