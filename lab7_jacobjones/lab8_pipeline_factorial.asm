main:
    # initialize n and go
    addi $a0, $zero, 0x1
    lw $a2 0x10($zero)
    # addi $a2, $zero, 0x4
    addi $t7, $zero, $zero # nop
    addi $t7, $zero, $zero # nop
    addi $t7, $zero, $zero # nop
    addi $t7, $zero, $zero # nop
    addi $t7, $zero, $zero # nop
    # load n from gpi0
    lw $a2 0x10($zero)
    sw $a2, 0x0($zero)
    # lw $a1, 0x0($zero)
    sw $a0, 0x4($zero)
    # lw $a1, 0x4($zero)
    # add loop until done here
    # addi $v0, $zero, 0x1
    # addi $t0, $zero, 0x2
fact_loop:
    lw $v1, 0x8($zero)
    addi $v1, $v1, $zero
    beq $v1, $zero, fact_loop
fact_end:
    lw $a1, 0xc($zero)
    addi $t7, $zero, $zero # nop
    addi $t7, $zero, $zero # nop
    addi $t7, $zero, $zero # nop
    addi $t7, $zero, $zero # nop
    # output result on gpo0 and in DM
    sw $a1, 0x18($zero)
    sw $a1, 0x24($zero)
    addi $t7, $zero, $zero # nop
    addi $t7, $zero, $zero # nop
    addi $t7, $zero, $zero # nop
    addi $t7, $zero, $zero # nop
end: