
configed=0
need_check=1
need_scan=0
if [ "$1" == "070INC" ]
then
	echo "Use 070INC"
	configed=1
elif [ "$1" == "070INC_3UART" ] 
then
	echo "Use 070INC_3UART"
	configed=1
elif [ "$1" == "040INC" ] 
then
	echo "Use 040INC"
	configed=1
elif [ "$1" == "043INC" ] 
then
	echo "Use 043INC"
	configed=1
elif [ "$1" == "043INC_2UART" ] 
then
	echo "Use 043INC_2UART"
	configed=1
elif [ "$1" == "102INC" ] 
then
	echo "Use 102INC"
	configed=1
elif [ "$1" == "050INC" ] 
then
	echo "Use 050INC"
	configed=1
elif [ "$1" == "050INC_ALLINONE" ] 
then
	echo "Use 050INC_ALLINONE"
	configed=1
elif [ "$1" == "057INC" ] 
then
	echo "Use 057INC"
	configed=1
elif [ "$1" == "035INC" ] 
then
	echo "Use 035INC"
	configed=1
elif [ "$1" == "035TINC" ] 
then
	echo "Use 035TINC"
	configed=1
elif [ "$1" == "rmimg" ] 
then
	rm arch/arm/boot/uImage*
	configed=0
	need_check=0
else
	echo "----------------------------------------------------------------"
	echo "Please choose below type:"
	echo "035INC"
	echo "035TINC"
	echo "040INC"
	echo "043INC"
	echo "043INC_2UART"
	echo "050INC"
	echo "050INC_ALLINONE"
	echo "057INC"
    echo "070INC"
	echo "070INC_3UART"
	echo "102INC"
	echo "Example: ./mkuseconfig.sh 070INC"
	echo "In order to skip samkoonhmi.h md5sum check, use like:"
	echo "         ./mkuseconfig.sh 070INC AKHMI"
	echo "Others Type, use like:"
	echo "         ./mkuseconfig.sh 070INC DBN"
	echo "Remove all uImage files:"
	echo "         ./mkconfig.sh rmimg"
	need_check=0
    configed=0
fi

if [ $need_check == 1 ] && [ -z "$2" ]
then
	if [ -f AKHMI_TYPE.TXT ]
	then
		OLD_TYPE=$(cat AKHMI_TYPE.TXT)
		echo "----------------------------------------------------------------"
		echo "Old AKHMI config file is: $OLD_TYPE"
		echo "----------------------------------------------------------------"
		src_md5=$(md5sum arch/arm/mach-omap2/include/mach/samkoonhmi.h)
		dst_md5=$(md5sum $OLD_TYPE) 
		echo "$src_md5"
		echo "$dst_md5"
		echo "----------------------------------------------------------------"
		src_md5=$(md5sum arch/arm/mach-omap2/include/mach/samkoonhmi.h|awk '{print $1}')
		dst_md5=$(md5sum $OLD_TYPE|awk '{print $1}') 
		if [ $src_md5 == $dst_md5 ] 
		then
			echo "AKHMI config files check [OK]"
			need_scan=0
		else
			echo "AKHMI config files check [Failed]"
			need_scan=1
		fi
	else
		echo "----------------------------------------------------------------"
		echo "No AKHMI_TYPE.TXT found, try to scan ./akhmiconfigs"
		echo "----------------------------------------------------------------"
		src_md5=$(md5sum arch/arm/mach-omap2/include/mach/samkoonhmi.h)
		echo "$src_md5"
		src_md5=$(md5sum arch/arm/mach-omap2/include/mach/samkoonhmi.h|awk '{print $1}')
		fid_md5=$(md5sum akhmiconfigs/* > md5tmp.txt | grep $src_md5 md5tmp.txt|awk '{print $1}')
		if [ -z "$fid_md5" ] 
		then
			echo "----------------------------------------------------------------"
			echo "Error: ./arch/arm/mach-omap2/include/mach/samkoonhmi.h Changed"
			echo "Please update ./akhmiconfigs/*_samkoonhmi.h"
			configed=0
			need_scan=0
		else
			echo "$fid_md5"
			echo "AKHMI config files check [OK]"
			need_scan=0
		fi
	fi

	if [ $need_scan == 1 ] 
	then
		echo "Try to scan ./akhmiconfigs"
		src_md5=$(md5sum arch/arm/mach-omap2/include/mach/samkoonhmi.h)
		echo "$src_md5"
		src_md5=$(md5sum arch/arm/mach-omap2/include/mach/samkoonhmi.h|awk '{print $1}')
		fid_md5=$(md5sum akhmiconfigs/* > md5tmp.txt | grep $src_md5 md5tmp.txt|awk '{print $1}')
		if [ -z "$fid_md5" ] 
		then
			echo "----------------------------------------------------------------"
			echo "Error: ./arch/arm/mach-omap2/include/mach/samkoonhmi.h Changed"
			echo "Please update ./akhmiconfigs/*_samkoonhmi.h"
			configed=0
		else
			echo "$fid_md5"
			echo "AKHMI config files check [OK]"
		fi
	fi
fi


if [ "$configed" == "1" ] 
then
if [ "$1" == "070INC" ]
then
	cp akhmiconfigs/070INC_samkoonhmi.h arch/arm/mach-omap2/include/mach/samkoonhmi.h
	echo "akhmiconfigs/070INC_samkoonhmi.h" > AKHMI_TYPE.TXT
elif [ "$1" == "070INC_3UART" ] 
then
	cp akhmiconfigs/070INC_3UART_samkoonhmi.h arch/arm/mach-omap2/include/mach/samkoonhmi.h
	echo "akhmiconfigs/070INC_3UART_samkoonhmi.h" > AKHMI_TYPE.TXT
elif [ "$1" == "040INC" ] 
then
	cp akhmiconfigs/040INC_samkoonhmi.h arch/arm/mach-omap2/include/mach/samkoonhmi.h
	echo "akhmiconfigs/040INC_samkoonhmi.h" > AKHMI_TYPE.TXT
elif [ "$1" == "043INC" ] 
then
	cp akhmiconfigs/043INC_samkoonhmi.h arch/arm/mach-omap2/include/mach/samkoonhmi.h
	echo "akhmiconfigs/043INC_samkoonhmi.h" > AKHMI_TYPE.TXT
elif [ "$1" == "043INC_2UART" ] 
then
	cp akhmiconfigs/043INC_2UART_samkoonhmi.h arch/arm/mach-omap2/include/mach/samkoonhmi.h
	echo "akhmiconfigs/043INC_2UART_samkoonhmi.h" > AKHMI_TYPE.TXT
elif [ "$1" == "102INC" ] 
then
	cp akhmiconfigs/102INC_samkoonhmi.h arch/arm/mach-omap2/include/mach/samkoonhmi.h
	echo "akhmiconfigs/102INC_samkoonhmi.h" > AKHMI_TYPE.TXT
elif [ "$1" == "050INC" ] 
then
	cp akhmiconfigs/050INC_samkoonhmi.h arch/arm/mach-omap2/include/mach/samkoonhmi.h
	echo "akhmiconfigs/050INC_samkoonhmi.h" > AKHMI_TYPE.TXT
elif [ "$1" == "050INC_ALLINONE" ] 
then
	cp akhmiconfigs/050INC_ALLINONE_samkoonhmi.h arch/arm/mach-omap2/include/mach/samkoonhmi.h
	echo "akhmiconfigs/050INC_ALLINONE_samkoonhmi.h" > AKHMI_TYPE.TXT
elif [ "$1" == "057INC" ] 
then
	cp akhmiconfigs/057INC_samkoonhmi.h arch/arm/mach-omap2/include/mach/samkoonhmi.h
	echo "akhmiconfigs/057INC_samkoonhmi.h" > AKHMI_TYPE.TXT
elif [ "$1" == "035INC" ] 
then
	cp akhmiconfigs/035INC_samkoonhmi.h arch/arm/mach-omap2/include/mach/samkoonhmi.h
	echo "akhmiconfigs/035INC_samkoonhmi.h" > AKHMI_TYPE.TXT
elif [ "$1" == "035TINC" ] 
then
	cp akhmiconfigs/035TINC_samkoonhmi.h arch/arm/mach-omap2/include/mach/samkoonhmi.h
	echo "akhmiconfigs/035TINC_samkoonhmi.h" > AKHMI_TYPE.TXT
fi

if [ "$2" == "DBN" ] 
then
	echo "Use DBN config"
	perl -pi -e 's/\/\/\#define	DBN		1/\#define	DBN		1/g' ./arch/arm/mach-omap2/include/mach/samkoonhmi.h
fi

echo "-----------------------------------------------------------------"
grep "#define" arch/arm/mach-omap2/include/mach/samkoonhmi.h
echo "-----------------------------------------------------------------"
make ARCH=arm CROSS_COMPILE=arm-eabi- am335x_evm_android_defconfig
make ARCH=arm CROSS_COMPILE=arm-eabi- uImage -j4
if [ "$1" == "070INC" ]
then
	mv arch/arm/boot/uImage  arch/arm/boot/uImage_070INC
elif [ "$1" == "070INC_3UART" ] 
then
	mv arch/arm/boot/uImage  arch/arm/boot/uImage_070INC_3UART
elif [ "$1" == "040INC" ] 
then
	mv arch/arm/boot/uImage  arch/arm/boot/uImage_040INC
elif [ "$1" == "043INC" ] 
then
	mv arch/arm/boot/uImage  arch/arm/boot/uImage_043INC
elif [ "$1" == "043INC_2UART" ] 
then
	mv arch/arm/boot/uImage  arch/arm/boot/uImage_043INC_2UART
elif [ "$1" == "102INC" ] 
then
	mv arch/arm/boot/uImage  arch/arm/boot/uImage_102INC
elif [ "$1" == "050INC" ] 
then
	mv arch/arm/boot/uImage  arch/arm/boot/uImage_050INC
elif [ "$1" == "050INC_ALLINONE" ] 
then
	mv arch/arm/boot/uImage  arch/arm/boot/uImage_050INC_ALLINONE
elif [ "$1" == "057INC" ] 
then
	mv arch/arm/boot/uImage  arch/arm/boot/uImage_057INC
elif [ "$1" == "035INC" ] 
then
	mv arch/arm/boot/uImage  arch/arm/boot/uImage_035INC
elif [ "$1" == "035TINC" ] 
then
	mv arch/arm/boot/uImage  arch/arm/boot/uImage_035TINC
fi
ls -lh arch/arm/boot/uImage*
fi
