include $(shell rospack find mk)/cmake_stack.mk

distclean:
# 	make clean
	cd collabnav_gmapping && make clean && rm -fr bin