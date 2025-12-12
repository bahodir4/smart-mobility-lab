# GitHub Repository Setup (Step-by-step)

## 1) Create the repository
1. Go to GitHub → New repository
2. Name: `tb3-automation-ros2` (or any name)
3. Public or Private (depends on course policy)
4. Do NOT initialize with README (we already have one)

## 2) Initialize git locally and push
From inside the project folder:
```bash
git init
git add .
git commit -m "Initial commit: TurtleBot3 automation project"
git branch -M main
git remote add origin <YOUR_GITHUB_REPO_URL>.git
git push -u origin main
```

## 3) Add the demo video link
Recommended options:
- Upload to YouTube (Unlisted) and paste the link in:
  - README.md → “Demo Video” section
- Or upload to Google Drive and share link

Example README snippet:
```md
## Demo Video
- Video: https://youtu.be/XXXXXXXXXXX
```

## 4) Tag a release (optional but professional)
```bash
git tag -a v1.0 -m "Course submission v1.0"
git push origin v1.0
```

## 5) What to submit
- GitHub repository link
- Demo video link
- Reflection PDF/MD (as required by professor)
