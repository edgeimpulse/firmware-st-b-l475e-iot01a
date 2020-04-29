
#
# Updates the version of TensorFlow Lite for Microcontrollers to a particular commit,
# and includes CMSIS-NN headers. Must be run from edgeimpulse root.
#
set -e

CURRENT_DIR="$(pwd)"

TF_COMMIT='a9aa8cb2d7caf22d61e0c1b9c12b6e002c3d7bb9'

TEMP_DIR='/tmp/tensorflow-update'
TF_DIR=${TEMP_DIR}'/tensorflow-'${TF_COMMIT}

FIRMWARE_SOURCE_DIR='firmware/source'
FIRMWARE_TF_DIR=${FIRMWARE_SOURCE_DIR}'/edge-impulse-sdk/tensorflow'
FIRMWARE_3P_DIR=${FIRMWARE_SOURCE_DIR}'/edge-impulse-sdk/third_party'
FIRMWARE_CMSIS_DIR=${FIRMWARE_SOURCE_DIR}'/edge-impulse-sdk/CMSIS'

SOURCE_REFERENCE=$TEMP_DIR/tf-reference
SOURCE_CMSIS=$TEMP_DIR/tf-cmsis

CMSIS_DIRECTIVE='EI_CLASSIFIER_TFLITE_ENABLE_CMSIS_NN'

# Clear any temp dir
rm -rf ${TEMP_DIR}
mkdir ${TEMP_DIR}

# Grab the correct version of the TensorFlow repo
curl https://github.com/edgeimpulse/tensorflow/archive/${TF_COMMIT}.zip -L --output ${TEMP_DIR}/tensorflow.zip
unzip ${TEMP_DIR}/tensorflow.zip -d ${TEMP_DIR}

# Generate the reference and CMSIS-NN projects
cd $TF_DIR
make -f tensorflow/lite/micro/tools/make/Makefile generate_hello_world_make_project
cp -r tensorflow/lite/micro/tools/make/gen/osx_x86_64/prj/hello_world/make/tensorflow $SOURCE_REFERENCE
make -f tensorflow/lite/micro/tools/make/Makefile clean
make -f tensorflow/lite/micro/tools/make/Makefile TAGS=cmsis-nn generate_hello_world_make_project
cp -r tensorflow/lite/micro/tools/make/gen/osx_x86_64/prj/hello_world/make/tensorflow $SOURCE_CMSIS

# We're going to transform the CMSIS-NN version of the project into something we can use
# in Edge Impulse.
# Append the CMSIS-NN kernels to the reference kernels and surround both with ifdefs, so the
# developer can choose between them at compile time.
CMSIS_KERNELS=$SOURCE_CMSIS/lite/micro/kernels/cmsis-nn/*.cc
for f in $CMSIS_KERNELS
do
    echo "Copying $f and reference kernels"
    filename="$(basename -- $f)"
    # Copy this file to the base kernels directory, appending an ifdef
    new_file=$SOURCE_CMSIS/lite/micro/kernels/$filename
    touch $new_file
    echo "// Patched by Edge Impulse to include both reference and CMSIS-NN kernels" >> $new_file
    echo "#include \"../../../../classifier/ei_classifier_config.h\"" >> $new_file
    echo "#if $CMSIS_DIRECTIVE == 1\n" >> $new_file
    cat "$f" >> $new_file
    echo "\n#else" >> $new_file
    # Now append the reference kernel implementation
    ref_kernel_path=$SOURCE_REFERENCE/lite/micro/kernels/$filename
    cat $ref_kernel_path >> $new_file
    echo "\n#endif" >> $new_file
    echo "Copied to $new_file"
done

# Remove the original cmsis-nn kernels
rm -rf $SOURCE_CMSIS/lite/micro/kernels/cmsis-nn

# Remove unnecessary TensorFlow Lite files
rm $SOURCE_CMSIS/lite/micro/debug_log.cc
rm -rf $SOURCE_CMSIS/lite/micro/examples/hello_world

# Change back to original dir
cd $CURRENT_DIR

# Copy to the Edge Impulse firmware directory
rm -rf $FIRMWARE_TF_DIR
cp -r $SOURCE_CMSIS $FIRMWARE_TF_DIR

# Update the cmsis-nn project's third_party dir
rm -rf $FIRMWARE_3P_DIR
cp -r $TF_DIR/tensorflow/lite/micro/tools/make/gen/osx_x86_64/prj/hello_world/make/third_party $FIRMWARE_3P_DIR

# Update the CMSIS libraries
rm -rf $FIRMWARE_CMSIS_DIR
mkdir $FIRMWARE_CMSIS_DIR
echo "Created by update_tflite.sh" > $FIRMWARE_CMSIS_DIR/sources.txt
# CMSIS core library
mkdir $FIRMWARE_CMSIS_DIR/Core
cp -r $TF_DIR/tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/Core/Include $FIRMWARE_CMSIS_DIR/Core
# CMSIS-DSP library
mkdir $FIRMWARE_CMSIS_DIR/DSP
cp -r $TF_DIR/tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/DSP/Include $FIRMWARE_CMSIS_DIR/DSP
cp -r $TF_DIR/tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/DSP/PrivateInclude $FIRMWARE_CMSIS_DIR/DSP/PrivateInclude
cp -r $TF_DIR/tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/DSP/Source $FIRMWARE_CMSIS_DIR/DSP
# These .c files include other .c files, which breaks the build for mbed since it builds
# all of the files in the project and ends up with duplicate objects. Remove them.
rm $FIRMWARE_CMSIS_DIR/DSP/Source/BasicMathFunctions/BasicMathFunctions.c
rm $FIRMWARE_CMSIS_DIR/DSP/Source/CommonTables/CommonTables.c
rm $FIRMWARE_CMSIS_DIR/DSP/Source/ControllerFunctions/ControllerFunctions.c
rm $FIRMWARE_CMSIS_DIR/DSP/Source/FastMathFunctions/FastMathFunctions.c
rm $FIRMWARE_CMSIS_DIR/DSP/Source/MatrixFunctions/MatrixFunctions.c
rm $FIRMWARE_CMSIS_DIR/DSP/Source/StatisticsFunctions/StatisticsFunctions.c
rm $FIRMWARE_CMSIS_DIR/DSP/Source/TransformFunctions/TransformFunctions.c
rm $FIRMWARE_CMSIS_DIR/DSP/Source/BayesFunctions/BayesFunctions.c
rm $FIRMWARE_CMSIS_DIR/DSP/Source/ComplexMathFunctions/ComplexMathFunctions.c
rm $FIRMWARE_CMSIS_DIR/DSP/Source/DistanceFunctions/DistanceFunctions.c
rm $FIRMWARE_CMSIS_DIR/DSP/Source/FilteringFunctions/FilteringFunctions.c
rm $FIRMWARE_CMSIS_DIR/DSP/Source/SVMFunctions/SVMFunctions.c
rm $FIRMWARE_CMSIS_DIR/DSP/Source/SupportFunctions/SupportFunctions.c
# CMSIS-NN library
mkdir $FIRMWARE_CMSIS_DIR/NN
cp -r $TF_DIR/tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Include $FIRMWARE_CMSIS_DIR/NN
cp -r $TF_DIR/tensorflow/lite/micro/tools/make/downloads/cmsis/CMSIS/NN/Source $FIRMWARE_CMSIS_DIR/NN

# TensorFlow license
cp $TF_DIR/LICENSE $FIRMWARE_TF_DIR