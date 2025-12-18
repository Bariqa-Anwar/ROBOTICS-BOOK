---
sidebar_position : 3
---

# Docusaurus Project Setup

Docusaurus is a static site generator that helps you build optimized websites quickly. It's particularly well-suited for documentation, and we will use it to structure our "Physical AI & Humanoid Robotics" book. This section covers installing the Docusaurus CLI and creating your project.

## Prerequisites

*   **Node.js** : Version 18.0 or above.
*   **npm or Yarn** : Node.js package manager.
*   **Internet Connection** : Required for downloading packages.

## 1. Install Node.js and npm/Yarn

If you don't have Node.js and npm/Yarn installed, you can follow the official Node.js installation guide for your operating system. For Ubuntu/WSL :

```bash
# Update package list
sudo apt update
sudo apt install -y curl

# Install Node.js 18.x (LTS)
curl -fsSL https ://deb.nodesource.com/setup_18.x | sudo -E bash -
sudo apt install -y nodejs

# Verify installation
node -v
npm -v
```

## 2. Create a New Docusaurus Project

You can create a new Docusaurus project using the official CLI.

```bash
npx create-docusaurus@latest my-book-site classic
```

*   `npx create-docusaurus@latest` : This command executes the latest Docusaurus project initializer.
*   `my-book-site` : This will be the name of your project directory. You can choose any name.
*   `classic` : This specifies the use of the classic template, which is a good starting point for documentation sites.

This command will :
1. Create a new directory `my-book-site`.
2. Install all necessary dependencies.
3. Generate a basic Docusaurus website structure.

## 3. Navigate to Your Project and Start the Development Server

Change into your newly created project directory and start the development server to see your site in action.

```bash
cd my-book-site
npm run start
```

This will open a new browser tab at `http ://localhost :3000` showing your Docusaurus website. Any changes you make to your documentation files will automatically be reflected here.

## 4. Basic Docusaurus Configuration

The main configuration file is `docusaurus.config.js` in your project root. You'll primarily edit the `docs` and `navbar` sections.

### Sidebar Configuration

Docusaurus uses `_category_.json` files within your `docs` folders to define sidebar items and their order. We will manage this as we add our book modules and chapters.

## Verification

To verify your Docusaurus setup :

1.  Navigate to your Docusaurus project directory (e.g., `my-book-site`) :
    ```bash
    cd my-book-site
    ```
2.  Start the development server :
    ```bash
    npm run start
    ```
    You should see output indicating the development server has started and your site is accessible, typically at `http ://localhost :3000`.
