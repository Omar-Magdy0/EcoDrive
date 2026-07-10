# EcoDrive Agent Rules

## Primary Goal
Implement the requested feature with minimal changes while preserving architecture, determinism, portability, and embedded performance.

---

## Before writing code

1. Read all files directly involved.
2. Understand the architecture before modifying anything.
3. Never guess APIs or behavior.
4. Search for existing implementations before creating new ones.
5. Reuse existing abstractions whenever possible.

---
## Repository Layout

EcoDrive/
    application/
        High-level application logic

    middleware/
        Communication protocols
        Serialization
        DAQ
        IDV
        ABF

    core/
        Platform-independent utilities

    platform/
        Hardware-specific implementations

            stm32f4/
                Firmware

            host/
                Linux simulation and testing

    EcoTool/
        Desktop host application

## Design Rules

- Prefer extending existing modules over creating new ones.
- Keep interfaces stable.
- Avoid unnecessary abstraction.
- Do not introduce frameworks or large dependencies.
- Favor compile-time solutions over runtime allocation.
- Minimize RAM usage.
- Minimize flash usage.
- Keep ISR execution deterministic.
- No hidden dynamic allocation unless explicitly requested.

---

## Coding Style

- Match existing naming conventions.
- Match formatting of surrounding code.
- Avoid duplicate code.
- Prefer constexpr/static inline when appropriate.
- Keep functions small and single-purpose.
- Avoid macros unless already used by the project.

---

## Embedded Rules

Assume this project targets STM32 unless stated otherwise.

Always consider:

- interrupt safety
- DMA interaction
- cache coherency (where applicable)
- USB throughput
- memory alignment
- stack usage
- timing determinism

Never introduce unnecessary copies.

---

## Architecture Rules

Respect existing layers.

Application
↓
Middleware
↓
Platform
↓
Driver

Never bypass a layer unless explicitly instructed.

Platform-specific code must remain inside platform folders.

---

## Performance Rules

Every change should consider:

- CPU cycles
- RAM usage
- Flash size
- Latency
- Throughput

Avoid premature optimization, but avoid obvious inefficiencies.

---

## Before Adding Code

Ask:

- Does this already exist?
- Can I reuse it?
- Can I simplify instead?
- Is there a cleaner extension point?

---

## Bug Fix Rules

Fix the root cause.

Never patch symptoms.

Do not silence warnings without understanding them.

---

## Refactoring Rules

Refactor only if it improves:

- readability
- maintainability
- correctness
- performance

Do not perform cosmetic refactors during unrelated tasks.

---

## Documentation

When adding a feature:

- explain the design briefly
- explain important tradeoffs
- document public APIs

Avoid redundant comments.

Code should explain itself.

---

## Communication

When finished always report:

1. Files modified.
2. Reason for each modification.
3. Possible side effects.
4. Remaining limitations.
5. Suggested next improvements.

---

## If Requirements Are Ambiguous

Never invent behavior.

State assumptions clearly.

If a decision affects architecture, stop and ask.

---

## Forbidden

Do not:

- rewrite unrelated files
- rename APIs unnecessarily
- change formatting globally
- introduce dependencies
- break backward compatibility
- remove existing functionality unless requested

---

## Preferred Mindset

Act as a senior embedded systems engineer working on a production firmware.

Priorities:

Correctness
>
Architecture
>
Maintainability
>
Performance
>
Convenience


## Repository Understanding

Before making changes:

1. Use the Repomix XML digest to understand project structure. located at repomix.zip.xml
2. Use the digest to locate candidate files.
3. Only inspect source files that are relevant to the requested task.
4. Never scan the repository blindly if the digest provides sufficient indexing.
5. Treat the digest as read-only documentation of the repository.

## Using the Repomix Digest

The digest is the preferred way to:

- locate classes
- locate functions
- understand dependencies
- understand module ownership
- identify file locations

Only open full source files after identifying them through the digest.

Do not repeatedly search the repository if the digest already contains sufficient information.

Use the digest for:

- architecture
- navigation
- locating code
- dependency discovery

Do not rely solely on the digest for:

- exact implementations
- debugging
- line-level edits

Always inspect the original source before modifying code.

The host and MCU based implementations expose equivalent functionality whenever possible.
When working on host-only code (silgui, platform/host), relax MCU and real time constraints.
Changes affecting one platform should consider whether the other implementation also requires modification.

Do not break interface compatibility.