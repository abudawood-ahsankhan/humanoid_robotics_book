# Humanoid Robotics Book - Docusaurus Website

This is the documentation website for the Humanoid Robotics Book, built with Docusaurus.

## Prerequisites

- Node.js (version 18 or higher)
- npm or yarn package manager

## Installation

First, install the required dependencies:

```bash
npm install
# or
yarn install
```

## Local Development

```bash
npm start
# or
yarn start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

## Build

```bash
npm run build
# or
yarn build
```

This command generates static content into the `build` directory and can be served using any static hosting service.

## Deployment

Using SSH:

```bash
USE_SSH=true npm run deploy
```

Not using SSH:

```bash
GIT_USER=<Your GitHub username> npm run deploy
```

## Project Structure

- `docs/`: Documentation pages in Markdown format
- `blog/`: Blog posts
- `src/`: Source files (components, CSS, etc.)
- `static/`: Static assets (images, files to be served as-is)
- `docusaurus.config.ts`: Main configuration file
- `sidebars.ts`: Navigation sidebar configuration

## Documentation Pages

The website includes comprehensive documentation about:

1. **Introduction**: Overview of the humanoid robotics book
2. **ROS 2 Fundamentals**: Core concepts of ROS 2
3. **Python Integration**: Using Python with ROS 2
4. **Robot Modeling**: Creating robot models with URDF
5. **Development Workflow**: Best practices for ROS 2 development
6. **Spec-Driven Development**: Specification-driven approach to robotics
7. **Conclusion**: Next steps and resources

## Contributing

1. Fork the repository
2. Create a new branch for your changes
3. Add or modify documentation as needed
4. Test your changes locally
5. Submit a pull request

## Troubleshooting

If you encounter issues during installation:

1. Make sure you have the correct Node.js version (18+)
2. Clear npm cache: `npm cache clean --force`
3. Delete node_modules and reinstall: `rm -rf node_modules && npm install`
4. Check your npm registry settings: `npm config get registry`