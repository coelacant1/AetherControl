git submodule update --remote --merge
cd ProtoTracer
git fetch --all
git reset --hard origin/main
git checkout main
git pull
cd ..
git add .
