package frc.chargers.utils.math.equations.ejml

import org.ejml.equation.Equation
import org.ejml.equation.ParseError
import org.ejml.equation.Sequence

public fun Equation.compileMultiline(equation: String, assignment: Boolean = true, debug: Boolean = false): Sequence =
    equation.lineSequence()
        .filterNot { it.all(Char::isWhitespace) }
        .mapIndexed { i, line ->
            try {
                compile(line, assignment, debug)
            } catch (e: Exception) {
                throw ParseError("Error on line $i (\"$line\"):\n${e.message}").also { it.initCause(e) }
            }
        }
        .reduce { a, b -> a + b }

public fun Equation.processMultiline(equation: String, assignment: Boolean = true, debug: Boolean = false): Equation = apply {
    compileMultiline(equation, assignment, debug).perform()
}