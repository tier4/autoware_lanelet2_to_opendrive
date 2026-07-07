"""Lift an ANTLR ``osc_file`` parse tree into the :mod:`.ast_model` IR.

The walker here is deliberately grammar-tolerant: instead of relying on fixed
child positions, it navigates by grammar rule name (looked up through the
parser's ``ruleNames`` table).  This keeps the extractor resilient to the many
single-child wrapper rules the OpenSCENARIO 2 grammar threads expressions
through, and to minor grammar revisions.
"""

from __future__ import annotations

from typing import Any, Optional

from .ast_model import (
    OscArgument,
    OscComparison,
    OscComposition,
    OscDoMember,
    OscElapsed,
    OscEnum,
    OscEventCond,
    OscField,
    OscFieldAccess,
    OscInvocation,
    OscModifier,
    OscProgram,
    OscScenario,
    OscStruct,
    OscWait,
)
from .parser import ParseResult
from .values import (
    BoolValue,
    FloatValue,
    IntValue,
    OscValue,
    StringValue,
    SymbolValue,
    parse_physical_literal,
)

# Rule names that carry a directly usable literal value.
_LITERAL_RULES = {
    "integer_literal",
    "float_literal",
    "physical_literal",
    "string_literal",
    "bool_literal",
}


class _TreeWalker:
    """Grammar-rule-aware navigation over an ANTLR parse tree."""

    def __init__(self, rule_names: list[str]) -> None:
        self._rule_names = rule_names
        from antlr4.tree.Tree import TerminalNodeImpl

        self._terminal_cls = TerminalNodeImpl

    def is_terminal(self, node: Any) -> bool:
        return isinstance(node, self._terminal_cls)

    def rule(self, node: Any) -> Optional[str]:
        """Return *node*'s grammar rule name, or ``None`` for a terminal."""
        if self.is_terminal(node):
            return None
        return self._rule_names[node.getRuleIndex()]

    def children(self, node: Any) -> list[Any]:
        return list(getattr(node, "children", None) or [])

    def children_by_rule(self, node: Any, name: str) -> list[Any]:
        """Return direct children of *node* whose rule name is *name*."""
        return [c for c in self.children(node) if self.rule(c) == name]

    def first_child_by_rule(self, node: Any, name: str) -> Optional[Any]:
        for child in self.children(node):
            if self.rule(child) == name:
                return child
        return None

    def find_descendant(self, node: Any, name: str) -> Optional[Any]:
        """Depth-first search for the first descendant with rule *name*."""
        for child in self.children(node):
            if self.rule(child) == name:
                return child
            found = self.find_descendant(child, name)
            if found is not None:
                return found
        return None

    def has_token(self, node: Any, text: str) -> bool:
        """Return ``True`` if any direct child is a terminal equal to *text*."""
        return any(
            self.is_terminal(c) and c.getText() == text for c in self.children(node)
        )

    def text(self, node: Any) -> str:
        return node.getText()


class _Extractor:
    """Stateful walker that produces an :class:`OscProgram`."""

    def __init__(self, walker: _TreeWalker) -> None:
        self._w = walker

    # -- expressions / values ------------------------------------------

    def evaluate(self, node: Any) -> OscValue:
        """Reduce an ``expression`` (or any subtree) to an :class:`OscValue`.

        Only the literal / symbolic leaf cases the translator cares about are
        modelled precisely; anything more complex degrades to a
        :class:`SymbolValue` carrying the raw source text.
        """
        w = self._w
        for rule in _LITERAL_RULES:
            leaf = node if w.rule(node) == rule else w.find_descendant(node, rule)
            if leaf is not None:
                return self._literal(rule, w.text(leaf))
        symbol = w.find_descendant(node, "enum_value_reference")
        if symbol is not None:
            return SymbolValue(w.text(symbol))
        identifier = w.find_descendant(node, "identifier")
        if identifier is not None:
            return SymbolValue(w.text(identifier))
        return SymbolValue(w.text(node))

    def _literal(self, rule: str, text: str) -> OscValue:
        if rule == "integer_literal":
            return IntValue(int(text, 0))
        if rule == "float_literal":
            return FloatValue(float(text))
        if rule == "physical_literal":
            return parse_physical_literal(text)
        if rule == "string_literal":
            return StringValue(text.strip("\"'"))
        if rule == "bool_literal":
            return BoolValue(text == "true")
        raise AssertionError(f"unhandled literal rule {rule!r}")  # pragma: no cover

    # -- arguments -----------------------------------------------------

    def extract_arguments(self, node: Any) -> list[OscArgument]:
        """Extract an ``argument_list`` under *node* (a call-like rule)."""
        w = self._w
        arg_list = w.first_child_by_rule(node, "argument_list")
        if arg_list is None:
            return []
        arguments: list[OscArgument] = []
        for child in w.children(arg_list):
            rule = w.rule(child)
            if rule == "positional_argument":
                expr = w.first_child_by_rule(child, "expression") or child
                arguments.append(OscArgument(name=None, value=self.evaluate(expr)))
            elif rule == "named_argument":
                name_node = w.first_child_by_rule(child, "argument_name")
                expr = w.first_child_by_rule(child, "expression")
                if name_node is not None and expr is not None:
                    arguments.append(
                        OscArgument(name=w.text(name_node), value=self.evaluate(expr))
                    )
        return arguments

    def _actor_of(self, node: Any) -> Optional[str]:
        """Return the actor prefix (``ego`` in ``ego.drive()``), if any."""
        w = self._w
        actor_expr = w.first_child_by_rule(node, "actor_expression")
        if actor_expr is None:
            return None
        return w.text(actor_expr)

    # -- modifiers -----------------------------------------------------

    def extract_modifier(self, node: Any) -> OscModifier:
        w = self._w
        name_node = w.first_child_by_rule(node, "modifier_name")
        name = w.text(name_node) if name_node is not None else w.text(node)
        return OscModifier(
            name=name,
            actor=self._actor_of(node),
            arguments=self.extract_arguments(node),
        )

    def _extract_with_block(
        self, invocation_node: Any
    ) -> tuple[list[OscModifier], Optional[OscEventCond]]:
        """Extract a ``with:`` block's modifiers and optional ``until`` clause."""
        w = self._w
        with_decl = w.first_child_by_rule(invocation_node, "behavior_with_declaration")
        if with_decl is None:
            return [], None
        modifiers: list[OscModifier] = []
        until: Optional[OscEventCond] = None
        for member in w.children_by_rule(with_decl, "behavior_with_member"):
            mod_node = w.first_child_by_rule(member, "modifier_application")
            if mod_node is not None:
                modifiers.append(self.extract_modifier(mod_node))
                continue
            until_node = w.first_child_by_rule(member, "until_directive")
            if until_node is not None:
                until = self.extract_event_condition(until_node)
        return modifiers, until

    # -- event conditions (wait / until) -------------------------------

    def extract_event_condition(self, node: Any) -> Optional[OscEventCond]:
        """Extract an ``event_condition`` (``elapsed(...)`` or a comparison)."""
        w = self._w
        cond = w.find_descendant(node, "event_condition")
        scope = cond if cond is not None else node

        elapsed = w.find_descendant(scope, "elapsed_expression")
        if elapsed is not None:
            return OscElapsed(duration=self.evaluate(elapsed))

        comparison = self._find_comparison(scope)
        if comparison is None:
            return None
        children = [c for c in w.children(comparison) if w.rule(c) is not None]
        op_node = w.first_child_by_rule(comparison, "relational_op")
        if op_node is None or len(children) < 2:
            return None
        lhs_text = w.text(children[0])
        if "." not in lhs_text:
            return None
        actor, attr = lhs_text.split(".", 1)
        return OscComparison(
            lhs=OscFieldAccess(actor=actor, attr=attr),
            op=w.text(op_node),
            rhs=self.evaluate(children[-1]),
        )

    def _find_comparison(self, node: Any) -> Optional[Any]:
        """Find the first ``relation`` node that carries a ``relational_op``."""
        w = self._w
        if w.rule(node) == "relation" and w.first_child_by_rule(node, "relational_op"):
            return node
        for child in w.children(node):
            found = self._find_comparison(child)
            if found is not None:
                return found
        return None

    # -- do directive --------------------------------------------------

    def extract_do_member(
        self, node: Any, composition: Optional[str]
    ) -> Optional[OscDoMember]:
        """Extract a ``do_member`` into an invocation, wait, or composition."""
        w = self._w
        comp_node = w.first_child_by_rule(node, "composition")
        if comp_node is not None:
            return self._extract_composition(comp_node)

        wait_node = w.first_child_by_rule(node, "wait_directive")
        if wait_node is not None:
            cond = self.extract_event_condition(wait_node)
            return OscWait(condition=cond) if cond is not None else None

        invocation = w.first_child_by_rule(node, "behavior_invocation")
        if invocation is not None:
            return self._extract_invocation(invocation, composition)
        return None

    def _extract_composition(self, node: Any) -> OscComposition:
        w = self._w
        op_node = w.first_child_by_rule(node, "composition_operator")
        operator = w.text(op_node) if op_node is not None else "serial"
        members: list[OscDoMember] = []
        for member in w.children_by_rule(node, "do_member"):
            extracted = self.extract_do_member(member, composition=operator)
            if extracted is not None:
                members.append(extracted)
        return OscComposition(operator=operator, members=members)

    def _extract_invocation(
        self, node: Any, composition: Optional[str]
    ) -> OscInvocation:
        w = self._w
        behavior_node = w.first_child_by_rule(node, "behavior_name")
        behavior = w.text(behavior_node) if behavior_node is not None else ""
        modifiers, until = self._extract_with_block(node)
        return OscInvocation(
            behavior=behavior,
            actor=self._actor_of(node),
            arguments=self.extract_arguments(node),
            modifiers=modifiers,
            until=until,
            composition=composition,
        )

    # -- declarations --------------------------------------------------

    def extract_fields(self, container: Any) -> list[OscField]:
        """Extract every ``parameter_declaration`` reachable under *container*."""
        w = self._w
        fields: list[OscField] = []
        for member in w.children(container):
            param = self._find_parameter_declaration(member)
            if param is not None:
                fields.extend(self._extract_field(param))
        return fields

    def _find_parameter_declaration(self, member: Any) -> Optional[Any]:
        w = self._w
        if w.rule(member) == "parameter_declaration":
            return member
        field_decl = w.find_descendant(member, "parameter_declaration")
        return field_decl

    def _extract_field(self, param: Any) -> list[OscField]:
        w = self._w
        names = [w.text(n) for n in w.children_by_rule(param, "field_name")]
        if not names:
            return []
        type_node = w.first_child_by_rule(param, "type_declarator")
        type_name = None
        if type_node is not None:
            declared = w.find_descendant(type_node, "declared_type_name")
            type_name = w.text(declared) if declared is not None else w.text(type_node)
        default: Optional[OscValue] = None
        if w.has_token(param, "="):
            expr = w.first_child_by_rule(param, "expression")
            if expr is None:
                expr = w.find_descendant(param, "default_value")
            if expr is not None:
                default = self.evaluate(expr)
        return [OscField(name=n, type_name=type_name, default=default) for n in names]

    def extract_scenario(self, node: Any) -> OscScenario:
        w = self._w
        behavior_names = w.children_by_rule(node, "qualified_behavior_name")
        name = w.text(behavior_names[0]) if behavior_names else ""
        inherits: Optional[str] = None
        if w.has_token(node, "inherits") and len(behavior_names) > 1:
            inherits = w.text(behavior_names[1])

        fields: list[OscField] = []
        modifiers: list[OscModifier] = []
        do: Optional[OscDoMember] = None

        # Scenario body members appear either wrapped in ``scenario_member_decl``
        # (fields, modifiers) or as a direct ``behavior_specification`` child, so
        # iterate every rule child and dispatch on what it contains.
        for member in w.children(node):
            if w.rule(member) is None or w.rule(member) == "qualified_behavior_name":
                continue
            param = self._find_parameter_declaration(member)
            if param is not None:
                fields.extend(self._extract_field(param))
                continue
            do_directive = w.find_descendant(member, "do_directive")
            if do_directive is not None:
                do_member = w.first_child_by_rule(do_directive, "do_member")
                if do_member is not None:
                    do = self.extract_do_member(do_member, composition=None)
                continue
            mod_node = w.first_child_by_rule(member, "modifier_application")
            if mod_node is None:
                mod_node = w.find_descendant(member, "modifier_application")
            if mod_node is not None:
                modifiers.append(self.extract_modifier(mod_node))

        return OscScenario(
            name=name,
            inherits=inherits,
            fields=fields,
            modifiers=modifiers,
            do=do,
        )

    def extract_enum(self, node: Any) -> OscEnum:
        w = self._w
        name_node = w.first_child_by_rule(node, "enum_name") or w.first_child_by_rule(
            node, "qualified_identifier"
        )
        name = w.text(name_node) if name_node is not None else ""
        members = [
            w.text(m)
            for m in [
                w.find_descendant(em, "enum_member_name") or em
                for em in w.children_by_rule(node, "enum_member_decl")
            ]
        ]
        return OscEnum(name=name, members=members)

    def extract_struct(self, node: Any) -> OscStruct:
        w = self._w
        name_node = w.find_descendant(node, "struct_name") or w.find_descendant(
            node, "qualified_identifier"
        )
        name = w.text(name_node) if name_node is not None else ""
        return OscStruct(name=name, fields=self.extract_fields(node))

    def extract_program(self, tree: Any) -> OscProgram:
        w = self._w
        scenarios: list[OscScenario] = []
        enums: list[OscEnum] = []
        structs: list[OscStruct] = []
        imports: list[str] = []

        for decl in self._iter_declarations(tree):
            rule = w.rule(decl)
            if rule == "scenario_declaration":
                scenarios.append(self.extract_scenario(decl))
            elif rule == "enum_declaration":
                enums.append(self.extract_enum(decl))
            elif rule == "struct_declaration":
                structs.append(self.extract_struct(decl))

        for import_stmt in self._iter_rule(tree, "import_statement"):
            ref = w.find_descendant(import_stmt, "import_reference")
            imports.append(w.text(ref) if ref is not None else w.text(import_stmt))

        return OscProgram(
            scenarios=scenarios, enums=enums, structs=structs, imports=imports
        )

    def _iter_declarations(self, tree: Any) -> list[Any]:
        w = self._w
        declarations: list[Any] = []
        for main in w.children_by_rule(tree, "main_statement"):
            osc_decl = w.first_child_by_rule(main, "osc_declaration")
            container = osc_decl if osc_decl is not None else main
            for child in w.children(container):
                if w.rule(child) is not None:
                    declarations.append(child)
        return declarations

    def _iter_rule(self, tree: Any, rule: str) -> list[Any]:
        w = self._w
        found: list[Any] = []

        def _recurse(node: Any) -> None:
            for child in w.children(node):
                if w.rule(child) == rule:
                    found.append(child)
                _recurse(child)

        _recurse(tree)
        return found


def extract_program(parse_result: ParseResult) -> OscProgram:
    """Extract the :class:`OscProgram` IR from a :class:`ParseResult`."""
    walker = _TreeWalker(parse_result.rule_names)
    return _Extractor(walker).extract_program(parse_result.tree)
