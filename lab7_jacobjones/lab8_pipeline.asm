    addi	$t0, $zero, 0x20			# $t0 = $t1 + 0
    j 0x10
    addi $t7, $zero, $zero # nop
    addi $t7, $zero, $zero # nop
    addi	$t1, $zero, 1			# $t0 = $t1 + 0
    jr		$t0					# jump to $ra
    addi $t7, $zero, $zero # nop
    addi $t7, $zero, $zero # nop
    addi	$t1, $zero, 2			# $t0 = $t1 + 0
    jal 0x30
    addi $t7, $zero, $zero # nop
    addi $t7, $zero, $zero # nop
    addi	$t1, $zero, 2			# $t0 = $t1 + 0
    addi	$t0, $zero, 0x2			# $t0 = $t1 + 0
    addi	$t1, $zero, 0x4			# $t0 = $t1 + 0
    add  $t2, $t1, $t0
    sub  $t2, $t1, $t0
    and  $t2, $t1, $t0
    or  $t2, $t1, $t0
    slt  $t2, $t1, $t0
    slt  $t2, $t0, $t1
    sw $t1, 0x20($zero)
    lw $t2, 0x20($zero)
    addi $t7, $zero, $zero # nop
    addi $t7, $zero, $zero # nop
    addi $t7, $zero, $zero # nop
    beq $t0, $t1, branch_eq
    beq $t0, $t0, branch_eq
    addi $t7, $zero, $zero # nop
    addi $t7, $zero, $zero # nop
    addi $t7, $zero, $zero # nop
    addi $t7, $zero, $zero # nop
branch_eq:
    multu $t0, $t1
    mflo $t2
    mfhi $t2
    sll $t2, $t0, 0x1
    srl $t2, $t1, 0x2
    addi $t7, $zero, $zero # nop
    addi $t7, $zero, $zero # nop
    addi $t7, $zero, $zero # nop
    addi $t7, $zero, $zero # nop
    addi $t7, $zero, $zero # nop
end:


# Instructions to test
# *add, sub, and, or, slt, lw, sw, beq, *j
# multu, mfhi, mflo, *jr, *jal, sll, srl  
