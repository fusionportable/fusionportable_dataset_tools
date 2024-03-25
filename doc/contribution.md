# Contribution Guidelines

We're thrilled that you're interested in contributing to our project! 
To maintain the quality and coherence of the codebase, we ask that you follow these guidelines when contributing.

## 1. Follow the Structure and Example

Before adding your content, please take a moment to **review the existing structure and any examples** provided in the repository and then **closely follow**:

- **Project Structure**: Make sure your contribution is placed in the correct location. 
  ```shell script
  fusionportable_dataset_tools
  ├── README.md
  ```

- **Following Steps for your contribution**:
  1. Fork the repository to your own GitHub account.
  2. Create a new branch for your changes.
  3. Make your changes in the new branch.
  4. Submit a pull request to the main repository with a clear description of what you've added or changed.

## 2. Use `git submodule add` for Your Own Package

Please use the `git submodule add` command to add your own package into the project. Here's how you can do it:
Enter command and then push:
```shell script
git submodule add <repository-url> <path-to-include-your-package>
  - `<repository-url>` is the URL of your package's repository.
  - `<path-to-include-your-package>` is the location within our project where your package should be included.
git submodule init
git submodule update
```

Thank you for contributing!