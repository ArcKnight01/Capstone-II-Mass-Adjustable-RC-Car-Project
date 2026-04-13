if [ "$(uname -s)" = "Linux" ]; then
    source .venv/bin/activate
else
    source "$HOME/anaconda3/etc/profile.d/conda.sh"
    conda activate bwsi
fi
git pull
python Controller.py
git add . && git commit -m "update" && git push
echo "Finished running."
