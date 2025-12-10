from abc import ABC, abstractmethod
from typing import Any, Dict, List, Optional
import logging
from pydantic import BaseModel

logger = logging.getLogger(__name__)

class ToolResult(BaseModel):
    """
    Base result class for tool execution
    """
    success: bool
    data: Any = None
    error: Optional[str] = None
    metadata: Optional[Dict[str, Any]] = {}

class BaseTool(ABC):
    """
    Base class for all tools in the agent system
    """
    def __init__(self, name: str, description: str):
        self.name = name
        self.description = description
        self.logger = logging.getLogger(self.__class__.__name__)

    @abstractmethod
    async def execute(self, **kwargs) -> ToolResult:
        """
        Execute the tool with the given parameters

        Args:
            **kwargs: Tool-specific parameters

        Returns:
            ToolResult containing success status, data, and any errors
        """
        pass

    def validate_parameters(self, params: Dict[str, Any], required_params: List[str]) -> bool:
        """
        Validate that all required parameters are present

        Args:
            params: Dictionary of parameters to validate
            required_params: List of required parameter names

        Returns:
            True if all required parameters are present, False otherwise
        """
        for param in required_params:
            if param not in params or params[param] is None:
                self.logger.error(f"Missing required parameter: {param}")
                return False
        return True

    def format_result(self, success: bool, data: Any = None, error: Optional[str] = None, metadata: Optional[Dict[str, Any]] = None) -> ToolResult:
        """
        Format a tool result

        Args:
            success: Whether the tool execution was successful
            data: The result data
            error: Error message if execution failed
            metadata: Additional metadata

        Returns:
            ToolResult instance
        """
        if metadata is None:
            metadata = {}

        return ToolResult(
            success=success,
            data=data,
            error=error,
            metadata=metadata
        )

class ToolRegistry:
    """
    Registry to manage and access tools
    """
    def __init__(self):
        self._tools = {}

    def register(self, tool: BaseTool):
        """
        Register a tool in the registry

        Args:
            tool: Tool instance to register
        """
        self._tools[tool.name] = tool
        logger.info(f"Registered tool: {tool.name}")

    def get_tool(self, name: str) -> Optional[BaseTool]:
        """
        Get a tool by name

        Args:
            name: Name of the tool to retrieve

        Returns:
            Tool instance if found, None otherwise
        """
        return self._tools.get(name)

    def list_tools(self) -> List[str]:
        """
        Get a list of all registered tool names

        Returns:
            List of tool names
        """
        return list(self._tools.keys())

    def execute_tool(self, name: str, **kwargs) -> Optional[ToolResult]:
        """
        Execute a tool by name

        Args:
            name: Name of the tool to execute
            **kwargs: Parameters for the tool

        Returns:
            ToolResult if tool exists, None otherwise
        """
        tool = self.get_tool(name)
        if not tool:
            logger.error(f"Tool not found: {name}")
            return self.format_error_result(f"Tool not found: {name}")

        try:
            return tool.execute(**kwargs)
        except Exception as e:
            logger.error(f"Error executing tool {name}: {str(e)}")
            return self.format_error_result(f"Error executing tool {name}: {str(e)}")

    def format_error_result(self, error_msg: str) -> ToolResult:
        """
        Format an error result

        Args:
            error_msg: Error message

        Returns:
            ToolResult with error information
        """
        return ToolResult(
            success=False,
            error=error_msg,
            metadata={"timestamp": __import__('time').time()}
        )

# Global tool registry instance
tool_registry = ToolRegistry()

def get_tool_registry() -> ToolRegistry:
    """
    Get the global tool registry instance
    """
    return tool_registry