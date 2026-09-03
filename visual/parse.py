import debugger # type: ignore
from dataclasses import dataclass
import numpy as np
import lldb # type: ignore

@dataclass
class Point2D:
    x: float
    y: float
    def to_list(self) -> list[float]:
        return [self.x, self.y]
    def __add__(self, other):
        return Point2D(self.x + other.x, self.y + other.y)

def parse_cv_vec4(frame, string : str):
    """
    Parses a cv vector 4f from the LLDB debugger and returns a list of its elements.
    """
    # vector_val = frame.EvaluateExpression(string)
    # if not vector_val.IsValid():
    #     print("DEBUG ERROR: Invalid cv vector 4f value!")
    #     return []

    # # Unwrap the vector value to get the underlying data structure
    # vector = debugger.unwrap(vector_val)
    # elements = []

    # for i in range(4):
    #     # Evaluate each val[i] as an expression
    #     item_val = vector.EvaluateExpression(f"val[{i}]")
    #     if item_val.IsValid():
    #         # SBValue gives string representation (e.g., "1.5"), convert directly to float
    #         val_str = item_val.GetValue()
    #         if val_str:
    #             elements.append(float(val_str))
    #         else:
    #             print("DEBUG ERROR: Invalid cv vector 4f value!")
    #             return []
    #     else:
    #         print("DEBUG ERROR: Invalid cv vector 4f value!")
    #         return []
    # return elements

    v = frame.FindVariable(string)
    if not v.IsValid():
        print(f"DEBUG ERROR: Invalid variable '{string}'!")
        return

    load_addr = v.GetLoadAddress()
    if load_addr == lldb.LLDB_INVALID_ADDRESS:
        print(f"DEBUG ERROR: Invalid load address for variable '{string}'!")
        return []
    
    num_bytes = 16 

    process = frame.GetThread().GetProcess()

    error = lldb.SBError()
    memory = process.ReadMemory(load_addr, num_bytes, error)

    return np.frombuffer(memory, dtype=np.float32).reshape((4,))

def parse_vector(frame, string : str, dimensions : list[int] = None):
    
    size_val = frame.EvaluateExpression(string + ".size()")
    size = size_val.GetValueAsUnsigned()
    address = frame.FindVariable(string).GetChildAtIndex(0).GetLoadAddress()
    if dimensions is not None:
        total_bytes = int(np.prod(dimensions) * size *4)  # 4 bytes per float32
    else:
        total_bytes = int(4 * size)  # 4 bytes per float32
    process = frame.GetThread().GetProcess()
    error = lldb.SBError()
    memory = process.ReadMemory(address, total_bytes, error)
    dim = [size] + dimensions if dimensions is not None else [size]
    return np.frombuffer(memory, dtype=np.float32).reshape(dim)