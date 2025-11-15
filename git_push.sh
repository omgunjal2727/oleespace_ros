#!/bin/bash

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}=== OleeSpace ROS Git Helper ===${NC}\n"

# Check if we're in a git repo
if ! git rev-parse --git-dir > /dev/null 2>&1; then
    echo -e "${YELLOW}Not a git repository. Initializing...${NC}"
    git init
    echo -e "${GREEN}Git repository initialized!${NC}\n"
fi

# Show current status
echo -e "${BLUE}Current Git Status:${NC}"
git status

echo -e "\n${BLUE}Changed/New Files:${NC}"
git status --short

# Show differences
echo -e "\n${YELLOW}Do you want to see detailed changes? (y/n)${NC}"
read -r show_diff
if [[ $show_diff == "y" ]]; then
    git diff
fi

# Add files
echo -e "\n${YELLOW}Add all changes? (y/n)${NC}"
read -r add_all

if [[ $add_all == "y" ]]; then
    git add .
    echo -e "${GREEN}All files staged!${NC}"
else
    echo -e "${YELLOW}Enter files to add (space-separated, or 'skip'):${NC}"
    read -r files_to_add
    if [[ $files_to_add != "skip" ]]; then
        git add $files_to_add
        echo -e "${GREEN}Files staged!${NC}"
    fi
fi

# Show what will be committed
echo -e "\n${BLUE}Files to be committed:${NC}"
git status --short

# Commit
echo -e "\n${YELLOW}Enter commit message:${NC}"
read -r commit_msg

if [[ -z "$commit_msg" ]]; then
    commit_msg="Update project files"
fi

git commit -m "$commit_msg"

# Check if remote exists
if ! git remote | grep -q origin; then
    echo -e "\n${YELLOW}No remote 'origin' found. Enter your GitHub repo URL:${NC}"
    echo -e "${BLUE}(e.g., https://github.com/YOUR_USERNAME/oleespace_ros.git)${NC}"
    read -r repo_url
    git remote add origin "$repo_url"
    echo -e "${GREEN}Remote 'origin' added!${NC}"
fi

# Check current branch
current_branch=$(git branch --show-current)
if [[ -z "$current_branch" ]]; then
    current_branch="main"
    git branch -M main
fi

# Push
echo -e "\n${YELLOW}Push to remote? (y/n)${NC}"
read -r do_push

if [[ $do_push == "y" ]]; then
    echo -e "${BLUE}Pushing to origin/$current_branch...${NC}"
    git push -u origin "$current_branch"
    
    if [[ $? -eq 0 ]]; then
        echo -e "\n${GREEN}✓ Successfully pushed to GitHub!${NC}"
    else
        echo -e "\n${YELLOW}Push failed. You may need to:${NC}"
        echo "1. Set up GitHub authentication"
        echo "2. Run: git push -u origin $current_branch --force (if needed)"
    fi
fi

echo -e "\n${GREEN}=== Done! ===${NC}"
