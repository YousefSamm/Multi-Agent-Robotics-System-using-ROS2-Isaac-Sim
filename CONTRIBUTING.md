# Contributing to Multi-Agent Robotic System

Thank you for your interest in contributing to this project! This document provides guidelines and instructions for contributing.

## How to Contribute

### Reporting Issues

If you find a bug or have a feature request:

1. Check if the issue already exists in the [Issues](https://github.com/YousefSamm/Multi-Agent-Robotis-System-using-ROS2-Isaac-Sim/issues) section
2. If not, create a new issue with:
   - Clear title and description
   - Steps to reproduce (for bugs)
   - Expected vs actual behavior
   - System information (OS, ROS2 version, etc.)

### Contributing Code

1. **Fork the repository**
   ```bash
   git clone --recurse-submodules https://github.com/YOUR_USERNAME/Multi-Agent-Robotis-System-using-ROS2-Isaac-Sim.git
   ```

2. **Create a feature branch**
   ```bash
   git checkout -b feature/your-feature-name
   ```

3. **Make your changes**
   - Follow the existing code style
   - Add comments for complex logic
   - Update documentation if needed
   - Test your changes thoroughly

4. **Commit your changes**
   ```bash
   git commit -m "Add: Description of your changes"
   ```
   Use clear, descriptive commit messages.

5. **Push to your fork**
   ```bash
   git push origin feature/your-feature-name
   ```

6. **Create a Pull Request**
   - Provide a clear description of your changes
   - Reference any related issues
   - Include screenshots/videos if applicable

## Code Style Guidelines

### C++ Code
- Follow ROS2 C++ style guide
- Use meaningful variable and function names
- Add comments for complex algorithms
- Keep functions focused and small

### Python Code
- Follow PEP 8 style guide
- Use type hints where appropriate
- Add docstrings to functions and classes
- Keep line length under 100 characters

### ROS2 Packages
- Follow ROS2 package structure conventions
- Include proper `package.xml` and `CMakeLists.txt`
- Add launch files for easy testing
- Include README.md for new packages

## Testing

Before submitting a pull request:

1. **Build the workspace**
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```

2. **Run tests** (if available)
   ```bash
   colcon test
   ```

3. **Test your changes manually**
   - Launch relevant nodes
   - Verify expected behavior
   - Check for errors in logs

## Documentation

- Update README.md if adding new features
- Add comments to complex code
- Update package-specific READMEs if needed
- Include usage examples

## Pull Request Process

1. Ensure your code follows the style guidelines
2. Make sure all tests pass
3. Update documentation as needed
4. Request review from maintainers
5. Address any feedback
6. Once approved, your PR will be merged

## Questions?

If you have questions about contributing:
- Open an issue with the `question` label
- Check existing documentation
- Review similar pull requests

Thank you for contributing! 🚀

