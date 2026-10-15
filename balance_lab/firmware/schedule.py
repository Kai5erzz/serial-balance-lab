"""Derive the host timing contract from the imported STM32 configuration.

Unrecognised clock/task changes fail explicitly instead of leaving a stale
desktop schedule running with a newer controller.
"""
from __future__ import annotations
import ast
import operator
import re
from pathlib import Path

DISPATCH_ORDER = ("ins", "motor", "chassis", "cmd", "gimbal")


def integer(expression: str) -> int:
    tree = ast.parse(re.sub(r"(?<=\d)[uUlL]+", "", expression), mode="eval")
    ops = {ast.Add: operator.add, ast.Sub: operator.sub, ast.Mult: operator.mul,
           ast.Div: operator.floordiv}
    def visit(node):
        if isinstance(node, ast.Constant) and isinstance(node.value, int):
            return node.value
        if isinstance(node, ast.BinOp) and type(node.op) in ops:
            return ops[type(node.op)](visit(node.left), visit(node.right))
        raise ValueError(f"unsupported clock expression: {expression}")
    return visit(tree.body)


def contract(root: Path) -> dict[str, int]:
    main = (root / "Core/Src/main.c").read_text()
    timer = (root / "Core/Src/tim.c").read_text()
    hal = (root / "Core/Inc/stm32h7xx_hal_conf.h").read_text()
    freertos = (root / "Core/Inc/FreeRTOSConfig.h").read_text()
    def value(pattern, source):
        found = re.search(pattern, source)
        if not found:
            raise ValueError(f"unrecognised firmware timing configuration: {pattern}")
        return integer(found[1].strip())
    if "RCC_PLLSOURCE_HSE" not in main or "RCC_SYSCLKSOURCE_PLLCLK" not in main:
        raise ValueError("host timing requires the imported HSE PLL clock")
    hse = value(r"#define\s+HSE_VALUE\s+\(([^)]+)\)", hal)
    pll = {n: value(r"\.PLL\.PLL" + n + r"\s*=\s*([^;]+);", main) for n in "MNP"}
    cpu = hse // pll["M"] * pll["N"] // pll["P"]
    sysdiv = value(r"SYSCLKDivider\s*=\s*RCC_SYSCLK_DIV(\d+)", main)
    ahbdiv = value(r"AHBCLKDivider\s*=\s*RCC_HCLK_DIV(\d+)", main)
    apbdiv = value(r"APB1CLKDivider\s*=\s*RCC_APB1_DIV(\d+)", main)
    cpu //= sysdiv
    timer_hz = cpu // ahbdiv // apbdiv * (2 if apbdiv > 1 else 1)
    psc = value(r"htim4.Init.Prescaler\s*=\s*([^;]+);", timer)
    arr = value(r"htim4.Init.Period\s*=\s*([^;]+);", timer)
    poll_numerator = (psc + 1) * (arr + 1) * 1_000_000
    if poll_numerator % timer_hz:
        raise ValueError("TIM4 period is not an integer number of microseconds")
    tick_hz = value(r"configTICK_RATE_HZ\s+\(\(TickType_t\)(\d+)\)", freertos)
    if 1_000_000 % tick_hz:
        raise ValueError("RTOS tick is not an integer number of microseconds")
    tick = 1_000_000 // tick_hz
    for path in ("chassis/chassis_task.c", "gimbal/gimbal_task.c", "motor/motor_task.c", "ins/ins_task.c"):
        text = (root / "src/task" / path).read_text(errors="replace")
        if not re.search(r"vTaskDelay(?:Until)?\([^;]*\b1\s*\);", text):
            raise ValueError(f"review changed task period: {path}")
    tasks = (root / "src/task/rm_task.h").read_text()
    # The desktop bridge runs the relevant equal-priority tasks in nominal
    # creation order. Do not silently keep that order after a firmware change.
    definitions = re.findall(r"osThreadDef\(\w+,\s*(\w+)_task_entry,\s*(\w+)", tasks)
    relevant = [(name, priority) for name, priority in definitions if name in DISPATCH_ORDER]
    if relevant != [(name, "osPriorityNormal") for name in DISPATCH_ORDER]:
        raise ValueError("review changed task priorities or creation order in rm_task.h")
    for name in DISPATCH_ORDER:
        body = re.search(r"void\s+" + name + r"_task_entry\(void const \*argument\)\s*\{(.*?)(?=__attribute__|#endif)", tasks, re.S)
        if not body or not re.search(r"vTaskDelayUntil\(&\w+,\s*1\s*\);", body[1]):
            raise ValueError(f"review changed outer task period: {name}")
    return {"control_tick_us": tick, "dm_poll_us": poll_numerator // timer_hz, "cpu_hz": cpu}
