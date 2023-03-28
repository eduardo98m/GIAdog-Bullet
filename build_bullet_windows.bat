@set ROOT=%cd%
@echo root= %ROOT%


rem pushd ThirdParty\bullet3 
pushd ThirdParty\bullet3Robotics
mkdir build_cmake
cd build_cmake

cmake -DUSE_MSVC_RUNTIME_LIBRARY_DLL=ON -DCMAKE_CXX_FLAGS="/MP" -DUSE_DOUBLE_PRECISION=ON -DCMAKE_DEBUG_POSTFIX="" -DINSTALL_LIBS=ON -DCMAKE_BUILD_TYPE=Release  -DCMAKE_INSTALL_PREFIX:PATH=local_install  ..

cmake  --build .  --target ALL_BUILD  --config Debug
cmake  --build .  --target INSTALL    --config Release

mkdir local_install\lib\Release
mkdir local_install\lib\Debug

copy  lib\Release local_install\lib\Release
copy  lib\Debug local_install\lib\Debug



